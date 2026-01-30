#pragma once
#include <windows.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

class ProcessUtils {
public:
    // [修改] 增加 stentTypeStr 参数
    static double runWorker(
        const std::string& workerExe,
        const std::string& meshRoot,
        const std::string& outputRoot,
        const std::string& stentTypeStr, // 新增：支架型号字符串
        const std::vector<double>& params,
        int timeoutMs)
    {
        std::string inputFile = "temp_in.txt";
        std::string outputFile = "temp_out.txt";
        double penalty = 1e9;

        // 1. 写参数到临时文件
        {
            std::ofstream out(inputFile);
            if (out.is_open()) {
                // [协议修改] 头部写入环境配置
                out << meshRoot << std::endl;
                out << outputRoot << std::endl;
                out << stentTypeStr << std::endl; // 写入支架型号

                // 写入优化参数
                out << params.size() << std::endl;
                for (double p : params) out << p << " ";
                out.close();
            }
            else {
                std::cerr << "[Process] Failed to write input file." << std::endl;
                return penalty;
            }
        }

        DeleteFileA(outputFile.c_str());

        // 2. 组装命令
        std::string cmd = workerExe + " " + inputFile + " " + outputFile;

        // 3. 启动进程
        STARTUPINFOA si;
        PROCESS_INFORMATION pi;
        ZeroMemory(&si, sizeof(si));
        si.cb = sizeof(si);
        si.dwFlags = STARTF_USESHOWWINDOW;
        si.wShowWindow = SW_SHOWDEFAULT;

        std::vector<char> cmdBuf(cmd.begin(), cmd.end());
        cmdBuf.push_back(0);

        BOOL success = CreateProcessA(NULL, cmdBuf.data(), NULL, NULL, FALSE, CREATE_NEW_CONSOLE, NULL, NULL, &si, &pi);

        if (!success) {
            std::cerr << "[Process] Failed to start SimWorker." << std::endl;
            return penalty;
        }

        // 4. 等待
        DWORD waitResult = WaitForSingleObject(pi.hProcess, timeoutMs);

        if (waitResult == WAIT_TIMEOUT) {
            std::cout << " [Timeout] SimWorker stuck! Killing process..." << std::endl;
            TerminateProcess(pi.hProcess, 1);
        }
        else {
            std::ifstream in(outputFile);
            if (in.is_open()) {
                in >> penalty;
            }
        }

        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
        return penalty;
    }
};