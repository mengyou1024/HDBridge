#pragma once

#include "../TOFDPort.h"
#include <HDBridge.h>
#include <cstdint>
#include <string>

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
#endif

using std::string;

class TOFDUSBPort : public HDBridge {
public:
    int          id   = {};
    std::wstring name = {};

    TOFDUSBPort() = default;

    virtual ~TOFDUSBPort() {
        if (isOpen()) {
            close();
        }
    }

    virtual bool open() override;
    virtual bool isOpen() override;
    virtual bool close() override;
    virtual bool isDeviceExist() override;

    virtual bool setSoundVelocity(float velocity) override;
    virtual bool setFrequency(int freq) override;
    virtual bool setVoltage(HB_Voltage voltage) override;
    virtual bool setChannelFlag(uint32_t flag) override;
    virtual bool setScanIncrement(int scanIncrement) override;
    virtual bool setLED(int ledStatus) override;
    virtual bool setDamperFlag(int damperFlag) override;
    virtual bool setEncoderPulse(int encoderPulse) override;
    virtual bool setZeroBias(int channel, float zero_us);
    virtual bool setPulseWidth(int channel, float pulseWidth) override;
    virtual bool setDelay(int channel, float delay_us) override;
    virtual bool setSampleDepth(int channel, float sampleDepth) override;
    virtual bool setSampleFactor(int channel, int sampleFactor) override;
    virtual bool setGain(int channel, float gain) override;
    virtual bool setFilter(int channel, HB_Filter filter) override;
    virtual bool setDemodu(int channel, HB_Demodu demodu) override;
    virtual bool setPhaseReverse(int channel, int reverse) override;
    virtual bool setGateInfo(int channel, const HB_GateInfo &info) override;
    virtual bool setGate2Type(int channel, HB_Gate2Type type) override;

    /**
     * @brief 复位编码器
     * @param coder
     * @return
     * @note param coder is not used
     */
    virtual bool resetCoder(int coder) override;
    virtual bool flushSetting() override;

    [[nodiscard]]
    virtual unique_ptr<HDBridge::NM_DATA> readDatas() override;

    [[nodiscard]]
    TOFDPort::NM_DATA *readRawDatas();
    void               freeRawDatas(TOFDPort::NM_DATA *data);

    virtual bool getCoderValue(int &coder0, int &coder1) override;
    virtual bool getCoderValueZ(int &coderZ0, int &coderZ1, int &coderF0, int &coderF1, int &coderC0, int &coderC1) override;

#ifdef USE_SQLITE_ORM

    #ifndef ORM_DB_NAME
    static constexpr std::string_view ORM_DB_NAME = "TOFDUSBPort.db";
    #endif // !ORM_DB_NAME

    static auto storage() {
        using namespace sqlite_orm;
        return make_storage(std::string(ORM_DB_NAME),
                            make_table("TOFDUSBPort",
                                       make_column("ID", &TOFDUSBPort::id, primary_key()),
                                       make_column("NAME", &TOFDUSBPort::name, unique()),
                                       make_column("CACHE", &TOFDUSBPort::mCache)));
    }

#endif
};
