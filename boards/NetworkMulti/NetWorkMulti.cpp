#include "NetworkMulti.h"
#include <HDBridge/NetworkMulti.h>

namespace HDBridge {

    bool NetworkMulti::open() {
        int _index = UNION_PORT_Open(const_cast<char *>(m_ipFPGA.c_str()), m_portFPGA, const_cast<char *>(m_ipPC.c_str()), m_portPC);
        if (_index > 0) {
            m_index = _index;
            return true;
        }
        return false;
    }

    bool NetworkMulti::isOpen() {
        return UNION_PORT_IsOpen(m_index);
    }

    bool NetworkMulti::close() {
        if (m_index > 0) {
            UNION_PORT_Close(m_index);
            return true;
        }
        return false;
    }

    bool NetworkMulti::setFrequency(int freq) {
        return UNION_PORT_SetFrequency(m_index, freq);
    }

    bool NetworkMulti::setVoltage(HB_Voltage voltage) {
        return UNION_PORT_SetVoltage(m_index, static_cast<int>(voltage));
    }

    bool NetworkMulti::setPulseWidth(int pulse) {
        return UNION_PORT_SetPulseWidth(m_index, pulse);
    }

    bool NetworkMulti::setDelay(int channel, float delay_us) {
        return UNION_PORT_SetDelay(m_index, channel, delay_us);
    }

    bool NetworkMulti::setSampleDepth(int channel, float sampleDepth) {
        return UNION_PORT_SetSampleDepth(m_index, channel, sampleDepth);
    }

    bool NetworkMulti::setSampleFactor(int channel, int sampleFactor) {
        return UNION_PORT_SetSampleFactor(m_index, channel, sampleFactor);
    }

    bool NetworkMulti::setGain(int channel, float gain) {
        return UNION_PORT_SetGain(m_index, channel, gain);
    }

    bool NetworkMulti::setFilter(int channel, HB_Filter filter) {
        return UNION_PORT_SetFilter(m_index, channel, static_cast<int>(filter));
    }

    bool NetworkMulti::setDemodu(int channel, HB_Demodu demodu) {
        return false;
    }

    bool NetworkMulti::setPhaseReverse(int channel, bool reverse) {
        return false;
    }

    bool NetworkMulti::setGateInfo(int channel, HB_GateInfo info) {
        return false;
    }

    bool NetworkMulti::setGate2Type(int channel, HB_Gate2Type type) {
        return false;
    }

    bool NetworkMulti::resetCoder(int coder) {
        return false;
    }

    bool NetworkMulti::flushSetting() {
        return false;
    }

    unique_ptr<HDBridge::NM_DATA> NetworkMulti::readDatas() {
        return nullptr;
    }

    unique_ptr<::NM_DATA> NetworkMulti::readRawDatas() {
        return nullptr;
    }
} // namespace HDBridge
