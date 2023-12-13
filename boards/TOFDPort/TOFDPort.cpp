#include "TOFDPort.h"
#include "HDBridge/TOFDPort.h"
#include <chrono>
#include <iostream>

#ifdef USE_SPDLOG
    #include <spdlog/spdlog.h>
#else
namespace spdlog {
    template <class... T>
    void debug(T &&...Args) {
        std::cout << "[debug]";
        (std::cout << ... << Args) << std::endl;
    }
    template <class... T>
    void info(T &&...Args) {
        std::cout << "[info]";
        (std::cout << ... << Args) << std::endl;
    }
    template <class... T>
    void warn(T... Args) {
        std::cout << "[warn]";
        (std::cout << ... << Args) << std::endl;
    }
} // namespace spdlog
#endif

using namespace TOFDPort;

bool TOFDUSBPort::open() {
    return TOFD_PORT_OpenDevice();
}

bool TOFDUSBPort::isOpen() {
    return TOFD_PORT_IsOpen();
}

bool TOFDUSBPort::close() {
    return TOFD_PORT_CloseDevice();
}

bool TOFDUSBPort::isDeviceExist() {
    return TOFD_PORT_IsDeviceExist();
}

bool TOFDUSBPort::setFrequency(int freq) {
    if (freq < 50) {
        freq = 50;
    }
    if (freq > 10000) {
        freq = 10000;
    }
    this->mCache.frequency = freq;
    return TOFD_PORT_SetFrequency(freq);
}

bool TOFDUSBPort::setVoltage(HB_Voltage voltage) {
    this->mCache.voltage = voltage;
    return TOFD_PORT_SetVoltage(static_cast<int>(voltage));
}

bool TOFDUSBPort::setChannelFlag(uint32_t flag) {
    if (flag == 0) {
        flag = 0xFFF0FFF;
    }
    this->mCache.channelFlag = flag;
    return TOFD_PORT_SetChannelFlag(static_cast<int>(flag));
}

bool TOFDUSBPort::setScanIncrement(int scanIncrement) {
    this->mCache.scanIncrement = scanIncrement;
    return TOFD_PORT_SetScanIncrement(scanIncrement);
}

bool TOFDUSBPort::setLED(int ledStatus) {
    this->mCache.ledStatus = ledStatus;
    return TOFD_PORT_SetLED(ledStatus);
}

bool TOFDUSBPort::setDamperFlag(int damperFlag) {
    this->mCache.damperFlag = damperFlag;
    return TOFD_PORT_SetDamperFlag(damperFlag);
}

bool TOFDUSBPort::setEncoderPulse(int encoderPulse) {
    this->mCache.encoderPulse = encoderPulse;
    return TOFD_PORT_SetEncoderPulse(encoderPulse);
}

bool TOFDUSBPort::setSoundVelocity(int channel, float velocity) {
    mCache.soundVelocity[channel % CHANNEL_NUMBER] = velocity;
    return true;
}

bool TOFDUSBPort::setZeroBias(int channel, float zero_us) {
    mCache.zeroBias[channel % CHANNEL_NUMBER] = zero_us;
    return TOFD_PORT_SetDelay(channel % CHANNEL_NUMBER, zero_us + mCache.delay[channel % CHANNEL_NUMBER]);
}

bool TOFDUSBPort::setPulseWidth(int channel, float pulseWidth) {
    if (pulseWidth < 30) {
        pulseWidth = 30;
    }
    this->mCache.pulseWidth[channel % CHANNEL_NUMBER] = pulseWidth;
    return TOFD_PORT_SetPulseWidth(channel % CHANNEL_NUMBER, pulseWidth);
}

bool TOFDUSBPort::setDelay(int channel, float delay_us) {
    this->mCache.delay[channel % CHANNEL_NUMBER] = delay_us;
    return TOFD_PORT_SetDelay(channel % CHANNEL_NUMBER, delay_us + mCache.zeroBias[channel % CHANNEL_NUMBER]);
}

bool TOFDUSBPort::setSampleFactor(int channel, int sampleFactor) {
    if (sampleFactor < 1) {
        sampleFactor = 1;
    } else if (sampleFactor > 255) {
        sampleFactor = 255;
    }
    this->mCache.sampleFactor[channel % CHANNEL_NUMBER] = sampleFactor;
    return TOFD_PORT_SetSampleFactor(channel % CHANNEL_NUMBER, sampleFactor);
}

bool TOFDUSBPort::setSampleDepth(int channel, float sampleDepth) {
    this->mCache.sampleDepth[channel % CHANNEL_NUMBER] = sampleDepth;
    return TOFD_PORT_SetSampleDepth(channel % CHANNEL_NUMBER, sampleDepth + mCache.zeroBias[channel % CHANNEL_NUMBER]);
}

bool TOFDUSBPort::setGain(int channel, float gain) {
    this->mCache.gain[channel % CHANNEL_NUMBER] = gain;
    return TOFD_PORT_SetGain(channel % CHANNEL_NUMBER, gain);
}

bool TOFDUSBPort::setFilter(int channel, HB_Filter filter) {
    this->mCache.filter[channel % CHANNEL_NUMBER] = filter;
    return TOFD_PORT_SetFilter(channel % CHANNEL_NUMBER, static_cast<int>(filter));
}

bool TOFDUSBPort::setDemodu(int channel, HB_Demodu demodu) {
    this->mCache.demodu[channel % CHANNEL_NUMBER] = demodu;
    return TOFD_PORT_SetDemodu(channel % CHANNEL_NUMBER, static_cast<int>(demodu));
}

bool TOFDUSBPort::setPhaseReverse(int channel, int reverse) {
    this->mCache.phaseReverse[channel % CHANNEL_NUMBER] = reverse;
    return TOFD_PORT_SetPhaseReverse(channel % CHANNEL_NUMBER, reverse);
}

bool TOFDUSBPort::setGateInfo(int channel, const HB_GateInfo &info) {
    if (info.gate == 0) {
        this->mCache.gateInfo[channel % CHANNEL_NUMBER] = info;
    } else {
        this->mCache.gate2Info[channel % CHANNEL_NUMBER] = info;
    }
    return TOFD_PORT_SetGateInfo(channel % CHANNEL_NUMBER, info.gate, info.active, info.alarmType, info.pos, info.width, info.height);
}

bool TOFDUSBPort::setGate2Type(int channel, HB_Gate2Type type) {
    this->mCache.gate2Type[channel % CHANNEL_NUMBER] = type;
    return TOFD_PORT_SetGate2Type(channel % CHANNEL_NUMBER, static_cast<int>(type));
}

bool TOFDUSBPort::resetCoder([[maybe_unused]] int coder) {
    return TOFD_PORT_ResetCoder_Immediate();
}

bool TOFDUSBPort::flushSetting() {
    return TOFD_PORT_FlushSetting();
}

unique_ptr<HDBridge::NM_DATA> TOFDUSBPort::readDatas() {
    auto data = readRawDatas();
    if (!data) {
        return nullptr;
    }

    auto ret        = make_unique<HDBridge::NM_DATA>();
    ret->iChannel   = data->iChannel;
    ret->iPackage   = data->iPackage;
    ret->iAScanSize = data->iAScanSize;
    ret->pAscan.resize(data->iAScanSize);
    memcpy(ret->pAscan.data(), data->pAscan, ret->iAScanSize);
    memcpy(ret->pCoder.data(), data->pCoder, sizeof(int32_t) * 2);
    memcpy(ret->pGatePos.data(), data->pGatePos, sizeof(int32_t) * 2);
    memcpy(ret->pGateAmp.data(), data->pGateAmp, sizeof(uint8_t) * 2);
    memcpy(ret->pAlarm.data(), data->pAlarm, sizeof(int32_t) * 2);

    freeRawDatas(data);
    return ret;
}

TOFDPort::NM_DATA *TOFDUSBPort::readRawDatas() {
    return TOFD_PORT_ReadDatasFormat();
}

void TOFDUSBPort::freeRawDatas(TOFDPort::NM_DATA *data) {
    TOFD_PORT_Free_NM_DATA(data);
}

bool TOFDUSBPort::getCoderValue(int &coder0, int &coder1) {
    return TOFD_PORT_GetCoderValue(&coder0, &coder1);
}

bool TOFDUSBPort::getCoderValueZ(int &coderZ0, int &coderZ1, int &coderF0, int &coderF1, int &coderC0, int &coderC1) {
    return TOFD_PORT_GetCoderValueZ(&coderZ0, &coderZ1, &coderF0, &coderF1, &coderC0, &coderC1);
}
