// Utils/PathUtils.h
#pragma once
#include <windows.h>
#include <string>
#include <algorithm>

class PathUtils {
public:
    // 获取当前 EXE 所在的目录 (末尾带 /)
    static std::string getExeDir() {
        char buffer[MAX_PATH];
        // 获取全路径 (e.g., C:\App\Optimizer.exe)
        GetModuleFileNameA(NULL, buffer, MAX_PATH);
        std::string path(buffer);

        // 去掉文件名，保留目录
        std::string::size_type pos = path.find_last_of("\\/");
        std::string dir = path.substr(0, pos);

        // 统一转为正斜杠 / 并确保末尾有 /
        for (char& c : dir) if (c == '\\') c = '/';
        if (!dir.empty() && dir.back() != '/') dir += '/';

        return dir;
    }

    // 路径标准化工具
    static std::string normalize(std::string path) {
        for (char& c : path) if (c == '\\') c = '/';
        if (!path.empty() && path.back() != '/') path += '/';
        return path;
    }
};