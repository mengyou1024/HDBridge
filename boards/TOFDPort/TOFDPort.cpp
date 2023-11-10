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
    if (TOFD_PORT_SetFrequency(freq)) {
        this->mCache.frequency = freq;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setVoltage(HB_Voltage voltage) {
    if (TOFD_PORT_SetVoltage(static_cast<int>(voltage))) {
        this->mCache.voltage = voltage;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setChannelFlag(uint32_t flag) {
    if (flag == 0) {
        flag = 0xFFF0FFF;
    }
    if (TOFD_PORT_SetChannelFlag(static_cast<int>(flag))) {
        this->mCache.channelFlag = flag;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setScanIncrement(int scanIncrement) {
    if (TOFD_PORT_SetScanIncrement(scanIncrement)) {
        this->mCache.scanIncrement = scanIncrement;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setLED(int ledStatus) {
    if (TOFD_PORT_SetLED(ledStatus)) {
        this->mCache.ledStatus = ledStatus;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setDamperFlag(int damperFlag) {
    if (TOFD_PORT_SetDamperFlag(damperFlag)) {
        this->mCache.damperFlag = damperFlag;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setEncoderPulse(int encoderPulse) {
    if (TOFD_PORT_SetEncoderPulse(encoderPulse)) {
        this->mCache.encoderPulse = encoderPulse;
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
    if (TOFD_PORT_SetPulseWidth(channel, pulseWidth)) {
        this->mCache.pulseWidth[channel] = pulseWidth;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setDelay(int channel, float delay_us) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    if (TOFD_PORT_SetDelay(channel, delay_us + mCache.zeroBias)) {
        this->mCache.delay[channel] = delay_us;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setSampleFactor(int channel, int sampleFactor) {
    if (sampleFactor < 1) {
        sampleFactor = 1;
    }
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    if (TOFD_PORT_SetSampleFactor(channel, sampleFactor)) {
        this->mCache.sampleFactor[channel] = sampleFactor;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setSampleDepth(int channel, float sampleDepth) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    if (TOFD_PORT_SetSampleDepth(channel, sampleDepth + mCache.zeroBias)) {
        this->mCache.sampleDepth[channel] = sampleDepth;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setGain(int channel, float gain) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    if (TOFD_PORT_SetGain(channel, gain)) {
        this->mCache.gain[channel] = gain;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setFilter(int channel, HB_Filter filter) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    if (TOFD_PORT_SetFilter(channel, static_cast<int>(filter))) {
        this->mCache.filter[channel] = filter;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setDemodu(int channel, HB_Demodu demodu) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    if (TOFD_PORT_SetDemodu(channel, static_cast<int>(demodu))) {
        this->mCache.demodu[channel] = demodu;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setPhaseReverse(int channel, int reverse) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    if (TOFD_PORT_SetPhaseReverse(channel, reverse)) {
        this->mCache.phaseReverse[channel] = reverse;
        return true;
    }
    return false;
}

bool TOFDUSBPort::setGateInfo(int channel, const HB_GateInfo &info) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    if (TOFD_PORT_SetGateInfo(channel, info.gate, info.active, info.alarmType, info.pos, info.width, info.height)) {
        HB_GateInfo cp = info;
        if (info.gate == 0) {
            this->mCache.gateInfo[channel] = cp;
        } else {
            this->mCache.gate2Info[channel] = cp;
        }
        return true;
    }
    return false;
}

bool TOFDUSBPort::setGate2Type(int channel, HB_Gate2Type type) {
    if (channel >= CHANNEL_NUMBER) {
        return false;
    }
    if (TOFD_PORT_SetGate2Type(channel, static_cast<int>(type))) {
        this->mCache.gate2Type[channel] = type;
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
    memcpy(ret->pCoder, data->pCoder, sizeof(int32_t) * 2);
    memcpy(ret->pGatePos, data->pGatePos, sizeof(int32_t) * 2);
    memcpy(ret->pGateAmp, data->pGateAmp, sizeof(uint8_t) * 2);
    memcpy(ret->pAlarm, data->pAlarm, sizeof(int32_t) * 2);

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
