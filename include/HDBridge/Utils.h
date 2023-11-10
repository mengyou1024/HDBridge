#pragma once

#include "../HDBridge.h"
#include <algorithm>
#include <functional>
#include <iostream>
#include <mutex>
#include <set>
#include <thread>
#include <type_traits>

#ifdef USE_SQLITE_ORM
    #include <sqlite_orm/sqlite_orm.h>
struct HD_ScanORM {
    shared_ptr<HDBridge::NM_DATA> mScanData[HDBridge::CHANNEL_NUMBER] = {};
};
namespace sqlite_orm {
    template <>
    struct type_printer<HD_ScanORM> : public blob_printer {};
    template <>
    struct statement_binder<HD_ScanORM> {
        int bind(sqlite3_stmt* stmt, int index, const HD_ScanORM& value) {
            std::vector<char> blobValue = {};
            for (auto i = 0; i < HDBridge::CHANNEL_NUMBER; i++) {
                auto temp = value.mScanData[i];
                if (temp == nullptr) {
                    temp           = std::make_shared<HDBridge::NM_DATA>();
                    temp->iChannel = i;
                }
                size_t lastSize = blobValue.size();
                blobValue.resize(blobValue.size() + sizeof(HDBridge::NM_DATA) - sizeof(vector<uint8_t>));
                memcpy(&blobValue[lastSize], &(temp->iChannel), sizeof(HDBridge::NM_DATA) - sizeof(vector<uint8_t>));
                if (temp->iAScanSize > 0 && temp->pAscan.size() > 0) {
                    lastSize = blobValue.size();
                    blobValue.resize(blobValue.size() + std::min((size_t)temp->iAScanSize, temp->pAscan.size()));
                    memcpy(&blobValue[lastSize], temp->pAscan.data(), std::min((size_t)temp->iAScanSize, temp->pAscan.size()));
                }
            }
            return statement_binder<std::vector<char>>().bind(stmt, index, blobValue);
        }
    };
    template <>
    struct field_printer<HD_ScanORM> {
        std::string operator()(const HD_ScanORM& value) {
            return {};
        }
    };
    template <>
    struct row_extractor<HD_ScanORM> {
        HD_ScanORM extract(sqlite3_stmt* stmt, int index) {
            char* blobPointer = (char*)sqlite3_column_blob(stmt, index);

            HD_ScanORM value;

            size_t pointerOffset = 0;

            for (auto i = 0; i < HDBridge::CHANNEL_NUMBER; i++) {
                value.mScanData[i] = std::make_shared<HDBridge::NM_DATA>();
                memcpy(&value.mScanData[i]->iChannel, &blobPointer[pointerOffset], sizeof(HDBridge::NM_DATA) - sizeof(vector<uint8_t>));
                pointerOffset += sizeof(HDBridge::NM_DATA) - sizeof(vector<uint8_t>);
                if (value.mScanData[i]->iAScanSize > 0) {
                    value.mScanData[i]->pAscan.resize(value.mScanData[i]->iAScanSize);
                    memcpy(value.mScanData[i]->pAscan.data(), &blobPointer[pointerOffset], value.mScanData[i]->iAScanSize);
                    pointerOffset += value.mScanData[i]->iAScanSize;
                }
            }
            return value;
        }
    };
} // namespace sqlite_orm
#endif

class HD_Utils {
public:
    int id = 0;

#ifdef USE_SQLITE_ORM
    using HD_ScanORM = ::HD_ScanORM;
#else
    struct HD_ScanORM {
        shared_ptr<HDBridge::NM_DATA> mScanData[HDBridge::CHANNEL_NUMBER] = {};
    };
#endif

    HD_ScanORM mScanOrm = {};

    [[deprecated("use `HD_Utils(std::unique_ptr<HDBridge> bridge)` insetead")]] explicit HD_Utils(HDBridge* bridge)
        : HD_Utils(std::unique_ptr<HDBridge>(bridge)) {}

    explicit HD_Utils(std::unique_ptr<HDBridge> bridge)
        : mBridge(std::move(bridge)) {
        if (mBridge) {
            mBridge->open();
        }
    }

    explicit HD_Utils(const HD_Utils& other)
        : HD_Utils() {
        *this = other;
    }

    HD_Utils& operator=(const HD_Utils& other) {
        id       = other.id;
        mScanOrm = other.mScanOrm;
        return *this;
    }

    explicit HD_Utils() = default;

    ~HD_Utils() {
        if (mBridge) {
            mBridge->close();
        }
        waitExit();
    }

    std::thread::id start();

    void setBridge(HDBridge* bridge) {
        mBridge = std::unique_ptr<HDBridge>(bridge);
    }

    template <class T>
    T getBridge() const {
        static_assert(std::is_pointer_v(T) && std::is_base_of_v(std::remove_pointer(T), HDBridge));
        return dynamic_cast<T>(mBridge.get());
    }

    HDBridge* getBridge() const {
        return mBridge.get();
    }

    void stop();
    void waitExit();

    void addReadCallback(const std::function<void(const HDBridge::NM_DATA&, const HD_Utils&)> callback);
    void removeReadCallback();

#ifdef USE_SQLITE_ORM

    #ifndef ORM_DB_NAME
    static constexpr std::string_view ORM_DB_NAME = "HD_Utils.db";
    #endif // !ORM_DB_NAME

    static auto storage() {
        using namespace sqlite_orm;
        return make_storage(std::string(ORM_DB_NAME),
                            make_table("HD_Utils",
                                       make_column("ID", &HD_Utils::id, primary_key()),
                                       make_column("Data", &HD_Utils::mScanOrm)));
    }

#endif

private:
    std::mutex                mScanDataMutex     = {};
    std::mutex                mReadCallbackMutex = {};
    bool                      mReadThreadExit    = false;
    std::unique_ptr<HDBridge> mBridge            = nullptr;

    std::vector<std::function<void(const HDBridge::NM_DATA&, const HD_Utils&)>> mReadCallback = {};

    std::thread mReadThread;

    void readThread();
};
