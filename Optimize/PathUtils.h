#pragma once
#include <string>
#include <algorithm>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#else
#include <limits.h>
#include <unistd.h>
#include <linux/limits.h>
#endif

class PathUtils {
public:
    // 获取当前 EXE 所在的目录 (末尾带 /)
    static std::string getExeDir() {
#ifdef _WIN32
        char buffer[MAX_PATH];
        GetModuleFileNameA(NULL, buffer, MAX_PATH);
        std::string path(buffer);
        std::string::size_type pos = path.find_last_of("\\/");
        std::string dir = path.substr(0, pos);
        for (char& c : dir) if (c == '\\') c = '/';
        if (!dir.empty() && dir.back() != '/') dir += '/';
        return dir;
#else
        char buffer[PATH_MAX];
        ssize_t count = readlink("/proc/self/exe", buffer, PATH_MAX);
        if (count == -1) return "./"; // fallback
        std::string path(buffer, count);
        std::string::size_type pos = path.find_last_of("/");
        std::string dir = path.substr(0, pos);
        if (!dir.empty() && dir.back() != '/') dir += '/';
        return dir;
#endif
    }

    // 路径标准化工具
    static std::string normalize(std::string path) {
        for (char& c : path) if (c == '\\') c = '/';
        if (!path.empty() && path.back() != '/') path += '/';
        return path;
    }
};