#pragma once
#include <windows.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

class ProcessUtils {
public:
    // 启动外部 Worker，超时则杀进程
    static double runWorker(const std::string& workerExe, const std::vector<double>& params, int timeoutMs) {
        std::string inputFile = "temp_in.txt";
        std::string outputFile = "temp_out.txt";
        double penalty = 1e6; // 失败/超时惩罚值

        // 1. 写参数到临时文件
        {
            std::ofstream out(inputFile);
            // 写入参数个数，方便读取校验
            out << params.size() << std::endl;
            for (double p : params) out << p << " ";
        }
        // 删除旧的输出文件，防止读取到上一次的结果
        DeleteFileA(outputFile.c_str());

        // 2. 组装命令: SimWorker.exe temp_in.txt temp_out.txt
        std::string cmd = workerExe + " " + inputFile + " " + outputFile;

        // 3. 启动进程
        STARTUPINFOA si;
        PROCESS_INFORMATION pi;
        ZeroMemory(&si, sizeof(si));
        si.cb = sizeof(si);
        ZeroMemory(&pi, sizeof(pi));

        // 显式要求显示窗口
        si.dwFlags = STARTF_USESHOWWINDOW;
        si.wShowWindow = SW_SHOWDEFAULT;

        std::vector<char> cmdBuf(cmd.begin(), cmd.end());
        cmdBuf.push_back(0);

        if (!CreateProcessA(NULL, cmdBuf.data(), NULL, NULL, FALSE, CREATE_NEW_CONSOLE, NULL, NULL, &si, &pi)) {
            std::cerr << "[Process] Failed to start SimWorker." << std::endl;
            return penalty;
        }

        // 4. 等待 (超时控制)
        DWORD waitResult = WaitForSingleObject(pi.hProcess, timeoutMs);

        if (waitResult == WAIT_TIMEOUT) {
            // 【超时】 -> 物理毁灭 (显存必释放)
            std::cout << " [Timeout] Killing process..." << std::endl;
            TerminateProcess(pi.hProcess, 1);
        }
        else {
            // 【正常退出】 -> 读取结果
            std::ifstream in(outputFile);
            if (in.is_open()) {
                in >> penalty; // 读取计算出的误差
            }
            else {
                std::cerr << " [Error] No output file generated." << std::endl;
            }
        }

        // 5. 清理句柄
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
        return penalty;
    }
};