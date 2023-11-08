#pragma once

#include <cstdint>
#include <memory>
#include <vector>

using std::make_shared;
using std::make_unique;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
class HDBridge {
public:
    enum class HB_Voltage : uint32_t {
        Voltage_50 = 0,
        Voltage_100V,
        Voltage_200V,
        Voltage_260V,
    };

    enum class HB_Filter : uint32_t {
        Filter_NONE = 0x00, ///< 不滤波
        Filter_2_5  = 0x01, ///< 2.5MHz
        Filter_5_0  = 0x02, ///< 5.0MHz
    };

    enum class HB_Demodu : uint32_t {
        Demodu_RF = 0x00, ///< 射频
        Demodu_Full,      ///< 全波
        Demodu_Positive,  ///< 正半波
        Demodu_Negative,  ///< 负半波
    };

    struct HB_GateInfo {
        int   gate      = {};
        int   active    = {};
        int   alarmType = {};
        float pos       = {};
        float width     = {};
        float height    = {};
    };

    enum class HB_Gate2Type : uint32_t {
        Fixed = 0, ///< 固定
        Follow,    ///< 跟随以波门1最高回波为零点
    };

    struct NM_DATA {
#pragma pack(1)
        int32_t  iChannel    = {}; ///< 通道号
        int32_t  iPackage    = {}; ///< 包序列
        int32_t  iAScanSize  = {}; ///< A扫长度
        int32_t  pCoder[2]   = {}; ///< 编码器值
        int32_t  pGatePos[2] = {}; ///< 波门位置
        int32_t  pAlarm[2]   = {}; ///< 波门报警
        uint8_t  pGateAmp[2] = {}; ///< 波门波幅
        uint16_t reserved    = {}; ///< 保留
#pragma pack()
        vector<uint8_t> pAscan = {}; // A扫数据
    };

    constexpr static int CHANNEL_NUMBER = 12; ///< 通道数

#pragma pack(1)
    struct cache_t {
        // cache
        int          frequency                    = {}; ///< 重复频率
        HB_Voltage   voltage                      = {}; ///< 发射电压
        uint32_t     channelFlag                  = {}; ///< 通道标志
        int          scanIncrement                = {}; ///< 扫查增量
        int          ledStatus                    = {}; ///< LED状态
        int          damperFlag                   = {}; ///< 阻尼标志
        int          encoderPulse                 = {}; ///< 编码器脉冲
        float        pulseWidth[CHANNEL_NUMBER]   = {}; ///< 脉冲宽度
        float        delay[CHANNEL_NUMBER]        = {}; ///< 延时
        float        sampleDepth[CHANNEL_NUMBER]  = {}; ///< 采样深度
        int          sampleFactor[CHANNEL_NUMBER] = {}; ///< 采样因子
        float        gain[CHANNEL_NUMBER]         = {}; ///< 增益
        HB_Filter    filter[CHANNEL_NUMBER]       = {}; ///< 滤波
        HB_Demodu    demodu[CHANNEL_NUMBER]       = {}; ///< 检波方式
        int          phaseReverse[CHANNEL_NUMBER] = {}; ///< 相位翻转
        HB_GateInfo  gateInfo[CHANNEL_NUMBER]     = {}; ///< 波门信息
        HB_Gate2Type gate2Type[CHANNEL_NUMBER]    = {}; ///< 波门2类型
    };

#pragma pack()

public:
    cache_t mCache = {};

public:
    HDBridge() {}

    virtual ~HDBridge() = default;

    virtual bool open()          = 0;
    virtual bool isOpen()        = 0;
    virtual bool close()         = 0;
    virtual bool isDeviceExist() = 0;

    virtual bool      setFrequency(int freq) = 0;
    virtual const int getFrequency() const final {
        return mCache.frequency;
    }

    virtual bool             setVoltage(HB_Voltage voltage) = 0;
    virtual const HB_Voltage getVoltage() const final {
        return mCache.voltage;
    }

    virtual bool       setChannelFlag(uint32_t flag) = 0;
    virtual const bool getChannelFlag() const final {
        return mCache.channelFlag;
    }

    virtual bool      setScanIncrement(int scanIncrement) = 0;
    virtual const int getScanIncrement() const final {
        return mCache.scanIncrement;
    }

    virtual bool       setLED(int ledStatus) = 0;
    virtual const bool getLED() const final {
        return mCache.ledStatus;
    };

    virtual bool      setDamperFlag(int damperFlag) = 0;
    virtual const int getDamperFlag() const final {
        return mCache.damperFlag;
    }

    virtual bool      setEncoderPulse(int encoderPulse) = 0;
    virtual const int getEncoderPulse() const final {
        return mCache.encoderPulse;
    }

    virtual bool                setPulseWidth(int channel, float pulseWidth) = 0;
    virtual const vector<float> getPulseWidth() const final {
        return vector<float>(mCache.pulseWidth, mCache.pulseWidth + CHANNEL_NUMBER);
    }

    virtual bool                setDelay(int channel, float delay_us) = 0;
    virtual const vector<float> getDelay() const final {
        return vector<float>(mCache.delay, mCache.delay + CHANNEL_NUMBER);
    }
    virtual bool                setSampleDepth(int channel, float sampleDepth) = 0;
    virtual const vector<float> getSampleDepth() const final {
        return vector<float>(mCache.sampleDepth, mCache.sampleDepth + CHANNEL_NUMBER);
    }
    virtual bool              setSampleFactor(int channel, int sampleFactor) = 0;
    virtual const vector<int> getSampleFactor() const final {
        return vector<int>(mCache.sampleFactor, mCache.sampleFactor + CHANNEL_NUMBER);
    }
    virtual bool                setGain(int channel, float gain) = 0;
    virtual const vector<float> getGain() const final {
        return vector<float>(mCache.gain, mCache.gain + CHANNEL_NUMBER);
    }
    virtual bool                    setFilter(int channel, HB_Filter filter) = 0;
    virtual const vector<HB_Filter> getFilter() const final {
        return vector<HB_Filter>(mCache.filter, mCache.filter + CHANNEL_NUMBER);
    }
    virtual bool                    setDemodu(int channel, HB_Demodu demodu) = 0;
    virtual const vector<HB_Demodu> getDemodu() const final {
        return vector<HB_Demodu>(mCache.demodu, mCache.demodu + CHANNEL_NUMBER);
    }
    virtual bool              setPhaseReverse(int channel, int reverse) = 0;
    virtual const vector<int> getPhaseReverse() const final {
        return vector<int>(mCache.phaseReverse, mCache.phaseReverse + CHANNEL_NUMBER);
    }
    virtual bool                      setGateInfo(int channel, const HB_GateInfo &info) = 0;
    virtual const vector<HB_GateInfo> getGateInfo() const final {
        return vector<HB_GateInfo>(mCache.gateInfo, mCache.gateInfo + CHANNEL_NUMBER);
    }
    virtual bool                       setGate2Type(int channel, HB_Gate2Type type) = 0;
    virtual const vector<HB_Gate2Type> getGate2Type() const final {
        return vector<HB_Gate2Type>(mCache.gate2Type, mCache.gate2Type + CHANNEL_NUMBER);
    }
    virtual bool resetCoder(int coder) = 0;
    virtual bool flushSetting()        = 0;

    virtual bool getCoderValue(int &coder0, int &coder1) = 0;

    virtual bool getCoderValueZ(int &coderZ0, int &coderZ1, int &coderF0, int &coderF1, int &coderC0, int &coderC1) = 0;

    [[nodiscard]]
    virtual unique_ptr<NM_DATA> readDatas() = 0;

    virtual void syncCache2Board() final {
        setFrequency(mCache.frequency);
        setVoltage(mCache.voltage);
        setChannelFlag(mCache.channelFlag);
        setScanIncrement(mCache.scanIncrement);
        setLED(mCache.ledStatus);
        setDamperFlag(mCache.damperFlag);
        setEncoderPulse(mCache.encoderPulse);
        for (int i = 0; i < CHANNEL_NUMBER; ++i) {
            setPulseWidth(i, mCache.pulseWidth[i]);
            setDelay(i, mCache.delay[i]);
            setSampleDepth(i, mCache.sampleDepth[i]);
            setSampleFactor(i, mCache.sampleFactor[i]);
            setGain(i, mCache.gain[i]);
            setFilter(i, mCache.filter[i]);
            setDemodu(i, mCache.demodu[i]);
            setPhaseReverse(i, mCache.phaseReverse[i]);
            setGateInfo(i, mCache.gateInfo[i]);
            setGate2Type(i, mCache.gate2Type[i]);
        }
        flushSetting();
    }
};
