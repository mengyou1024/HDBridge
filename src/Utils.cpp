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

void HD_Utils::pushCallback() {
    mCallbackStack.push(mReadCallback);
    removeReadCallback();
}

void HD_Utils::popCallback() {
    if (mCallbackStack.size() > 0) {
        std::lock_guard<std::mutex> lock(mReadCallbackMutex);
        mReadCallback = mCallbackStack.top();
        mCallbackStack.pop();
    }
}

void HD_Utils::autoGain(int channel, int gateIndex, float goal, float gainStep) {
    std::thread t([this, channel, gateIndex, goal, gainStep]() {
        volatile bool getGoal = false;
        float                                                                     lastGain = getBridge()->getGain(channel);
        std::function<void(const HDBridge::NM_DATA& data, const HD_Utils& utils)> f        = [&getGoal, lastGain, channel, gateIndex, goal, gainStep](const HDBridge::NM_DATA& data, const HD_Utils& utils) -> void {
            if (data.iChannel != channel) {
                return;
            }
            if (utils.getBridge()->getGateInfo(gateIndex, channel).width == 0) {
                getGoal = true;
                return;
            }
            float step = 0.f;
            float currentAmp = data.pGateAmp[gateIndex] / 255.f;

            if (currentAmp > goal) {
                step = -std::abs(gainStep);
            } else {
                step = std::abs(gainStep);
            }

            if (std::abs(currentAmp - goal) < 0.005f) {
                getGoal = true;
                return;
            }
            float gain = utils.getBridge()->getGain(channel);
            if (step > 0 && currentAmp > goal) {
                getGoal = true;
                return;
            } else if (step < 0 && currentAmp < goal) {
                getGoal = true;
                return;
            }
            if (gain > 110.f || gain < 0.f) {
                getGoal = true;
                utils.getBridge()->setGain(channel, lastGain);
                utils.getBridge()->flushSetting();
                return;
            }
            gain += step;
            utils.getBridge()->setGain(channel, gain);
            utils.getBridge()->flushSetting();
        };
        mReadCallbackMutex.lock();
        mReadCallback.push_back(f);
        mReadCallbackMutex.unlock();
        while (getGoal == false) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        mReadCallbackMutex.lock();
        mReadCallback.pop_back();
        mReadCallbackMutex.unlock();
    });
    t.join();
}

void HD_Utils::readThread() {
    while (mReadThreadExit == false) {
        if (!mBridge) {
            mReadThreadExit = true;
            break;
        }
        if (mReadCallback.size() == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        auto data = mBridge->readDatas();
        if (data) {
            std::lock_guard<std::mutex> readLock(mReadCallbackMutex);
            for (const auto& callback : mReadCallback) {
                callback(*data.get(), *this);
            }
            std::lock_guard<std::mutex> dataLock(mScanDataMutex);
            if (data->iChannel >= 0 && data->iChannel < HDBridge::CHANNEL_NUMBER) {
                mScanOrm.mScanData[data->iChannel] = std::move(data);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
