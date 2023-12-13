// #include <HDBridge.h>
// #include <HDBridge/TOFDPort.h>
// #include <HDBridge/Utils.h>

#include <HDBridge/TOFDPort.h>
#include <HDBridge/NetworkMulti.h>
#include <HDBridge/Utils.h>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <sqlite_orm/sqlite_orm.h>
#include <thread>

#ifdef _WIN32
    #include <Windows.h>
#endif

#ifndef ORM_DB_NAME
    #define ORM_DB_NAME "SystemConfig.db"
#endif // !ORM_DB_NAME

int main() {
    HD_Utils utils(std::unique_ptr<HDBridge>(new NetworkMulti));
    ORM_HDBridge::storage().sync_schema();
    try {
    ORM_HDBridge::storage().insert(*utils.getBridge());
    } catch(std::exception &e) {
        std::cout << e.what() << std::endl;
    }

    return 0;
}