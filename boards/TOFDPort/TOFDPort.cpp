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

bool TOFDUSBPort::setSoundVelocity(float velocity) {
    mCache.soundVelocity = velocity;
    return true;
}

bool TOFDUSBPort::setFrequency(int freq) {
    if (freq < 50) {
        freq = 50;
    }
    if (freq > 10000) {
        freq = 10000;
    }
    this->mCache.frequency = freq;
    if (TOFD_PORT_SetFrequency(freq)) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setVoltage(HB_Voltage voltage) {
    this->mCache.voltage = voltage;
    if (TOFD_PORT_SetVoltage(static_cast<int>(voltage))) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setChannelFlag(uint32_t flag) {
    if (flag == 0) {
        flag = 0xFFF0FFF;
    }
    this->mCache.channelFlag = flag;
    if (TOFD_PORT_SetChannelFlag(static_cast<int>(flag))) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setScanIncrement(int scanIncrement) {
    this->mCache.scanIncrement = scanIncrement;
    if (TOFD_PORT_SetScanIncrement(scanIncrement)) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setLED(int ledStatus) {
    this->mCache.ledStatus = ledStatus;
    if (TOFD_PORT_SetLED(ledStatus)) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setDamperFlag(int damperFlag) {
    this->mCache.damperFlag = damperFlag;
    if (TOFD_PORT_SetDamperFlag(damperFlag)) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setEncoderPulse(int encoderPulse) {
    this->mCache.encoderPulse = encoderPulse;
    if (TOFD_PORT_SetEncoderPulse(encoderPulse)) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setZeroBias(int channel, float zero_us) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    mCache.zeroBias[channel] = zero_us;
    if (TOFD_PORT_SetDelay(channel, zero_us + mCache.delay[channel])) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setPulseWidth(int channel, float pulseWidth) {
    if (pulseWidth < 30) {
        pulseWidth = 30;
    }
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    this->mCache.pulseWidth[channel] = pulseWidth;
    if (TOFD_PORT_SetPulseWidth(channel, pulseWidth)) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setDelay(int channel, float delay_us) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    this->mCache.delay[channel] = delay_us;
    if (TOFD_PORT_SetDelay(channel, delay_us + mCache.zeroBias[channel])) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setSampleFactor(int channel, int sampleFactor) {
    if (sampleFactor < 1) {
        sampleFactor = 1;
    } else if (sampleFactor > 255) {
        sampleFactor = 255;
    }
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    this->mCache.sampleFactor[channel] = sampleFactor;
    if (TOFD_PORT_SetSampleFactor(channel, sampleFactor)) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setSampleDepth(int channel, float sampleDepth) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    this->mCache.sampleDepth[channel] = sampleDepth;
    if (TOFD_PORT_SetSampleDepth(channel, sampleDepth + mCache.zeroBias[channel])) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setGain(int channel, float gain) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    this->mCache.gain[channel] = gain;
    if (TOFD_PORT_SetGain(channel, gain)) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setFilter(int channel, HB_Filter filter) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    this->mCache.filter[channel] = filter;
    if (TOFD_PORT_SetFilter(channel, static_cast<int>(filter))) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setDemodu(int channel, HB_Demodu demodu) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    this->mCache.demodu[channel] = demodu;
    if (TOFD_PORT_SetDemodu(channel, static_cast<int>(demodu))) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setPhaseReverse(int channel, int reverse) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    this->mCache.phaseReverse[channel] = reverse;
    if (TOFD_PORT_SetPhaseReverse(channel, reverse)) {
        return true;
    }
    return false;
}

bool TOFDUSBPort::setGateInfo(int channel, const HB_GateInfo &info) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    if (info.gate == 0) {
        this->mCache.gateInfo[channel] = info;
    } else {
        this->mCache.gate2Info[channel] = info;
    }
    if (TOFD_PORT_SetGateInfo(channel, info.gate, info.active, info.alarmType, info.pos, info.width, info.height)) {

        return true;
    }
    return false;
}

bool TOFDUSBPort::setGate2Type(int channel, HB_Gate2Type type) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    this->mCache.gate2Type[channel] = type;
    if (TOFD_PORT_SetGate2Type(channel, static_cast<int>(type))) {
        return true;
    }
    return false;
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
