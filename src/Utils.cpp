#include "HDBridge/Utils.h"
#include <chrono>

std::thread::id HD_Utils::start() {
    mReadThreadExit = false;
    mReadThread     = std::thread(&HD_Utils::readThread, this);
    return mReadThread.get_id();
}

void HD_Utils::stop() {
    mReadThreadExit = true;
}

void HD_Utils::waitExit() {
    stop();
    if (mReadThread.joinable()) {
        mReadThread.join();
    }
}

void HD_Utils::addReadCallback(const std::function<void(const HDBridge::NM_DATA&, const HD_Utils&)> callback) {
    std::lock_guard<std::mutex> lock(mReadCallbackMutex);
    if (callback) {
        mReadCallback.push_back(callback);
    }
}

void HD_Utils::removeReadCallback() {
    std::lock_guard<std::mutex> lock(mReadCallbackMutex);
    mReadCallback.clear();
}

void HD_Utils::readThread() {
    while (mReadThreadExit == false) {
        if (!mBridge) {
            mReadThreadExit = true;
            break;
        }
        auto data = mBridge->readDatas();
        if (data) {
            std::lock_guard<std::mutex> readLock(mReadCallbackMutex);
            for (auto& callback : mReadCallback) {
                callback(*data.get(), *this);
            }
            std::lock_guard<std::mutex> dataLock(mScanDataMutex);
            if (data->iChannel < HDBridge::CHANNEL_NUMBER) {
                mScanOrm.mScanData[data->iChannel] = std::move(data);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
