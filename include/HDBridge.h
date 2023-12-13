#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

using std::make_shared;
using std::make_unique;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

class HDBridge {
public:
    enum class HB_Voltage : uint32_t {
        Voltage_50V = 0,
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
        int32_t                iChannel     = {}; ///< 通道号
        int32_t                iPackage     = {}; ///< 包序列
        int32_t                iAScanSize   = {}; ///< A扫长度
        std::array<int32_t, 2> pCoder       = {}; ///< 编码器值
        std::array<int32_t, 2> pGatePos     = {}; ///< 波门位置
        std::array<int32_t, 2> pAlarm       = {}; ///< 波门报警
        std::array<uint8_t, 2> pGateAmp     = {}; ///< 波门波幅
        uint16_t               reserved     = {}; ///< 保留
#pragma pack()
        vector<uint8_t> pAscan = {}; // A扫数据
    };

    constexpr static int CHANNEL_NUMBER = 12; ///< 通道数

#pragma pack(1)
    struct cache_t {
        // cache
        int                                      frequency     = {}; ///< 重复频率
        HB_Voltage                               voltage       = {}; ///< 发射电压
        uint32_t                                 channelFlag   = {}; ///< 通道标志
        int                                      scanIncrement = {}; ///< 扫查增量
        int                                      ledStatus     = {}; ///< LED状态
        int                                      damperFlag    = {}; ///< 阻尼标志
        int                                      encoderPulse  = {}; ///< 编码器脉冲
        std::array<float, CHANNEL_NUMBER>        soundVelocity = {}; ///< 声速
        std::array<float, CHANNEL_NUMBER>        zeroBias      = {}; ///< 零点偏移
        std::array<float, CHANNEL_NUMBER>        pulseWidth    = {}; ///< 脉冲宽度
        std::array<float, CHANNEL_NUMBER>        delay         = {}; ///< 延时
        std::array<float, CHANNEL_NUMBER>        sampleDepth   = {}; ///< 采样深度
        std::array<int, CHANNEL_NUMBER>          sampleFactor  = {}; ///< 采样因子
        std::array<float, CHANNEL_NUMBER>        gain          = {}; ///< 增益
        std::array<HB_Filter, CHANNEL_NUMBER>    filter        = {}; ///< 滤波
        std::array<HB_Demodu, CHANNEL_NUMBER>    demodu        = {}; ///< 检波方式
        std::array<int, CHANNEL_NUMBER>          phaseReverse  = {}; ///< 相位翻转
        std::array<HB_GateInfo, CHANNEL_NUMBER>  gateInfo      = {}; ///< 波门信息
        std::array<HB_GateInfo, CHANNEL_NUMBER>  gate2Info     = {}; ///< 波门2信息
        std::array<HB_Gate2Type, CHANNEL_NUMBER> gate2Type     = {}; ///< 波门2类型
    };
#pragma pack()

public:
    int          id      = {};
    std::wstring name    = {};
    bool         isValid = {};
    cache_t      mCache  = {};

public:
    HDBridge() {
        for (auto &g : mCache.gate2Info) {
            g.gate = 1;
        }
        mCache.soundVelocity.fill(5920.0f);
    }

    virtual ~HDBridge() = default;

    HDBridge &operator=(const HDBridge &other) {
        id      = other.id;
        name    = other.name;
        isValid = other.isValid;
        mCache  = other.mCache;
        return *this;
    }

    virtual bool open() {
        return false;
    }
    virtual bool isOpen() {
        return false;
    }
    virtual bool close() {
        return false;
    }
    virtual bool isDeviceExist() {
        return false;
    }

    virtual bool setFrequency(int freq) {
        return false;
    }
    virtual const int getFrequency() const final {
        return mCache.frequency;
    }

    virtual bool setVoltage(HB_Voltage voltage) {
        return false;
    }
    virtual const HB_Voltage getVoltage() const final {
        return mCache.voltage;
    }

    virtual bool setChannelFlag(uint32_t flag) {
        return false;
    }
    virtual const uint32_t getChannelFlag() const final {
        return mCache.channelFlag;
    }

    virtual bool setScanIncrement(int scanIncrement) {
        return false;
    }
    virtual const int getScanIncrement() const final {
        return mCache.scanIncrement;
    }

    virtual bool setLED(int ledStatus) {
        return false;
    }
    virtual const bool getLED() const final {
        return mCache.ledStatus;
    };

    virtual bool setDamperFlag(int damperFlag) {
        return false;
    }
    virtual const int getDamperFlag() const final {
        return mCache.damperFlag;
    }

    virtual bool setEncoderPulse(int encoderPulse) {
        return false;
    }
    virtual const int getEncoderPulse() const final {
        return mCache.encoderPulse;
    }

    virtual bool setSoundVelocity(int channel, float velocity) {
        return false;
    }
    virtual const float getSoundVelocity(int channel) const final {
        return mCache.soundVelocity[channel % CHANNEL_NUMBER];
    }

    virtual bool setZeroBias(int channel, float zero_us) {
        return false;
    }
    virtual const float getZeroBias(int channel) const final {
        return mCache.zeroBias[channel % CHANNEL_NUMBER];
    }

    virtual bool setPulseWidth(int channel, float pulseWidth) {
        return false;
    }
    virtual const float getPulseWidth(int channel) const final {
        return mCache.pulseWidth[channel % CHANNEL_NUMBER];
    }

    virtual bool setDelay(int channel, float delay_us) {
        return false;
    }
    virtual const float getDelay(int channel) const final {
        return mCache.delay[channel % CHANNEL_NUMBER];
    }
    virtual bool setSampleDepth(int channel, float sampleDepth) {
        return false;
    }
    virtual const float getSampleDepth(int channel) const final {
        return mCache.sampleDepth[channel % CHANNEL_NUMBER];
    }
    virtual bool setSampleFactor(int channel, int sampleFactor) {
        return false;
    }
    virtual const int getSampleFactor(int channel) const final {
        return mCache.sampleFactor[channel % CHANNEL_NUMBER];
    }
    virtual bool setGain(int channel, float gain) {
        return false;
    }
    virtual const float getGain(int channel) const final {
        return mCache.gain[channel % CHANNEL_NUMBER];
    }
    virtual bool setFilter(int channel, HB_Filter filter) {
        return false;
    }
    virtual const HB_Filter getFilter(int channel) const final {
        return mCache.filter[channel % CHANNEL_NUMBER];
    }
    virtual bool setDemodu(int channel, HB_Demodu demodu) {
        return false;
    }
    virtual const HB_Demodu getDemodu(int channel) const final {
        return mCache.demodu[channel % CHANNEL_NUMBER];
    }
    virtual bool setPhaseReverse(int channel, int reverse) {
        return false;
    }
    virtual const int getPhaseReverse(int channel) const final {
        return mCache.phaseReverse[channel % CHANNEL_NUMBER];
    }
    virtual bool setGateInfo(int channel, const HB_GateInfo &info) {
        return false;
    }
    virtual const HB_GateInfo getGateInfo(int index, int channel) const final {
        if (index == 0) {
            return mCache.gateInfo[channel % CHANNEL_NUMBER];
        } else {
            return mCache.gate2Info[channel % CHANNEL_NUMBER];
        }
    }
    virtual bool setGate2Type(int channel, HB_Gate2Type type) {
        return false;
    }
    virtual const HB_Gate2Type getGate2Type(int channel) const final {
        return mCache.gate2Type[channel % CHANNEL_NUMBER];
    }
    virtual bool resetCoder(int coder) {
        return false;
    }
    virtual bool flushSetting() {
        return false;
    }

    virtual bool getCoderValue(int &coder0, int &coder1) {
        return false;
    }

    virtual bool getCoderValueZ(int &coderZ0, int &coderZ1, int &coderF0, int &coderF1, int &coderC0, int &coderC1) {
        return false;
    }

    [[nodiscard]]
    virtual unique_ptr<NM_DATA> readDatas() {
        return false;
    }

    virtual void syncCache2Board() final {
        setFrequency(mCache.frequency);
        setVoltage(mCache.voltage);
        setChannelFlag(mCache.channelFlag);
        setScanIncrement(mCache.scanIncrement);
        setLED(mCache.ledStatus);
        setDamperFlag(mCache.damperFlag);
        setEncoderPulse(mCache.encoderPulse);
        for (int i = 0; i < CHANNEL_NUMBER; ++i) {
            setSoundVelocity(i, mCache.soundVelocity[i]);
            setZeroBias(i, mCache.zeroBias[i]);
            setPulseWidth(i, mCache.pulseWidth[i]);
            setDelay(i, mCache.delay[i]);
            setSampleDepth(i, mCache.sampleDepth[i]);
            setSampleFactor(i, mCache.sampleFactor[i]);
            setGain(i, mCache.gain[i]);
            setFilter(i, mCache.filter[i]);
            setDemodu(i, mCache.demodu[i]);
            setPhaseReverse(i, mCache.phaseReverse[i]);
            setGateInfo(i, mCache.gateInfo[i]);
            setGateInfo(i, mCache.gate2Info[i]);
            setGate2Type(i, mCache.gate2Type[i]);
        }
        flushSetting();
    }

    virtual void defaultInit() final {
        setFrequency(1200);
        setVoltage(HB_Voltage::Voltage_100V);
        setChannelFlag(0xFFF0FFF);
        setScanIncrement(0);
        setLED(0);
        setDamperFlag(0xFFF);
        setEncoderPulse(1);
        for (int i = 0; i < CHANNEL_NUMBER; ++i) {
            setPulseWidth(i, 210.f);
            setZeroBias(i, static_cast<float>(distance2time(0.0, i)));
            setDelay(i, static_cast<float>(distance2time(0.0, i)));
            setSampleDepth(i, static_cast<float>(distance2time(200.0, i)));
            setSampleFactor(i, 13);
            setGain(i, 30.f);
            setFilter(i, static_cast<HB_Filter>(3));
            setDemodu(i, HB_Demodu::Demodu_Full);
            setPhaseReverse(i, 0);
            HB_GateInfo info = {
                0, 1, 0, 0.2f, 0.2f, 0.5f};
            setGateInfo(i, mCache.gateInfo[i]);
            info.gate   = 1;
            info.active = 1;
            setGateInfo(i, mCache.gateInfo[i]);
            setGate2Type(i, mCache.gate2Type[i]);
        }
        flushSetting();
    }

    /**
     * @brief 时间转距离
     * @param time_us 微秒时间
     * @param velocity_in_m_per_s 声速(m/s)
     * @return 距离 (mm)
     */
    static constexpr double time2distance(double time_us, double velocity_in_m_per_s) {
        return time_us * velocity_in_m_per_s / 2000.0;
    }

    /**
     * @brief 距离转时间
     * @param distance_mm
     * @param velocity_in_m_per_s
     * @return
     */
    static constexpr double distance2time(double distance_mm, double velocity_in_m_per_s) {
        if (velocity_in_m_per_s == 0.0) {
            return 0;
        } else {
            return distance_mm * 2000.0 / velocity_in_m_per_s;
        }
    }

    virtual double time2distance(double time_us, int channel) final {
        return time2distance(time_us, (double)mCache.soundVelocity[channel % CHANNEL_NUMBER]);
    }

    virtual double distance2time(double distance_mm, int channel) final {
        return distance2time(distance_mm, (double)mCache.soundVelocity[channel % CHANNEL_NUMBER]);
    }

    /*
     * @brief 通道参数复制参数
     * @param src 复制源
     * @param dist 复制目标列表
     */
    virtual void paramCopy(size_t src, std::vector<size_t> dist) {
        if (src >= static_cast<size_t>(CHANNEL_NUMBER)) {
            return;
        }
        auto         zeroBias     = mCache.zeroBias[src];
        auto         pluseWidth   = mCache.pulseWidth[src];
        auto         delay        = mCache.delay[src];
        auto         sampleDepth  = mCache.sampleDepth[src];
        auto         sampleFactor = mCache.sampleFactor[src];
        auto         gain         = mCache.gain[src];
        auto         filter       = mCache.filter[src];
        auto         demodu       = mCache.demodu[src];
        auto         phaseReverse = mCache.phaseReverse[src];
        HB_GateInfo  gateInfo     = mCache.gateInfo[src];
        HB_GateInfo  gate2Info    = mCache.gate2Info[src];
        HB_Gate2Type gate2Type    = mCache.gate2Type[src];
        for (auto i : dist) {
            mCache.zeroBias[i]     = zeroBias;
            mCache.pulseWidth[i]   = pluseWidth;
            mCache.delay[i]        = delay;
            mCache.sampleDepth[i]  = sampleDepth;
            mCache.sampleFactor[i] = sampleFactor;
            mCache.gain[i]         = gain;
            mCache.filter[i]       = filter;
            mCache.demodu[i]       = demodu;
            mCache.phaseReverse[i] = phaseReverse;
            mCache.gateInfo[i]     = gateInfo;
            mCache.gate2Info[i]    = gate2Info;
            mCache.gate2Type[i]    = gate2Type;
        }
        syncCache2Board();
    }
};

#ifdef USE_SQLITE_ORM
    #include <sqlite_orm/sqlite_orm.h>
namespace sqlite_orm {
    template <>
    struct type_printer<HDBridge::cache_t> : public blob_printer {};
    template <>
    struct statement_binder<HDBridge::cache_t> {
        int bind(sqlite3_stmt *stmt, int index, const HDBridge::cache_t &value) {
            std::vector<char> blobValue;
            blobValue.resize(sizeof(HDBridge::cache_t));
            blobValue.reserve(sizeof(HDBridge::cache_t));
            memcpy_s(blobValue.data(), blobValue.capacity(), &value, sizeof(HDBridge::cache_t));
            return statement_binder<std::vector<char>>().bind(stmt, index, blobValue);
        }
    };
    template <>
    struct field_printer<HDBridge::cache_t> {
        std::string operator()(const HDBridge::cache_t &value) {
            return {};
        }
    };
    template <>
    struct row_extractor<HDBridge::cache_t> {
        HDBridge::cache_t extract(sqlite3_stmt *stmt, int index) {
            auto              blobPointer = sqlite3_column_blob(stmt, index);
            HDBridge::cache_t value;
            memcpy(&value, blobPointer, sizeof(HDBridge::cache_t));
            return value;
        }
    };
} // namespace sqlite_orm

    #ifndef ORM_DB_NAME
static constexpr std::string_view ORM_DB_NAME = "HDBridge.db";
    #endif // !ORM_DB_NAME

namespace ORM_HDBridge {
    static auto storage() {
        using namespace sqlite_orm;
        return make_storage(std::string(ORM_DB_NAME),
                            make_table("HDBridge",
                                       make_column("ID", &HDBridge::id, primary_key()),
                                       make_column("NAME", &HDBridge::name, unique()),
                                       make_column("VALID", &HDBridge::isValid),
                                       make_column("CACHE", &HDBridge::mCache)));
    }
} // namespace ORM_HDBridge
#endif
