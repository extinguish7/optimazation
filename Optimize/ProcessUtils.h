#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#endif

class ProcessUtils {
public:
    static double runWorker(
        const std::string& workerExe,
        const std::string& meshRoot,
        const std::string& outputRoot,
        const std::string& stentTypeStr,
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
                out << meshRoot << std::endl;
                out << outputRoot << std::endl;
                out << stentTypeStr << std::endl;
                out << params.size() << std::endl;
                for (double p : params) out << p << " ";
                out.close();
            }
            else {
                std::cerr << "[Process] Failed to write input file." << std::endl;
                return penalty;
            }
        }

        // 删除旧的输出文件
#ifdef _WIN32
        DeleteFileA(outputFile.c_str());
#else
        unlink(outputFile.c_str());
#endif

#ifdef _WIN32
        // ================= Windows 实现 =================
        std::string cmd = workerExe + " " + inputFile + " " + outputFile;
        STARTUPINFOA si;
        PROCESS_INFORMATION pi;
        ZeroMemory(&si, sizeof(si));
        si.cb = sizeof(si);
        si.dwFlags = STARTF_USESHOWWINDOW;
        si.wShowWindow = SW_SHOWDEFAULT;

        std::vector<char> cmdBuf(cmd.begin(), cmd.end());
        cmdBuf.push_back(0);

        if (!CreateProcessA(NULL, cmdBuf.data(), NULL, NULL, FALSE, CREATE_NEW_CONSOLE, NULL, NULL, &si, &pi)) {
            std::cerr << "[Process] Failed to start SimWorker." << std::endl;
            return penalty;
        }

        DWORD waitResult = WaitForSingleObject(pi.hProcess, timeoutMs);
        if (waitResult == WAIT_TIMEOUT) {
            std::cout << " [Timeout] SimWorker stuck! Killing process..." << std::endl;
            TerminateProcess(pi.hProcess, 1);
        }
        else {
            std::ifstream in(outputFile);
            if (in.is_open()) in >> penalty;
        }
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);

#else
        // ================= Linux 实现 (fork + exec) =================
        pid_t pid = fork();

        if (pid == -1) {
            std::cerr << "[Process] Fork failed." << std::endl;
            return penalty;
        }
        else if (pid == 0) {
            // 子进程
            // execl 需要参数列表，第一个是路径，接下来的参数，最后 NULL
            execl(workerExe.c_str(), workerExe.c_str(), inputFile.c_str(), outputFile.c_str(), (char*)NULL);
            // 如果执行到这里说明 execl 失败
            perror("[Process] execl failed");
            exit(1);
        }
        else {
            // 父进程：带超时的等待
            auto start = std::chrono::steady_clock::now();
            int status;
            bool finished = false;

            while (true) {
                // WNOHANG: 非阻塞轮询
                pid_t result = waitpid(pid, &status, WNOHANG);
                if (result == pid) { // 子进程结束
                    finished = true;
                    break;
                }
                else if (result == -1) {
                    perror("waitpid");
                    break;
                }

                // 检查超时
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
                if (elapsed > timeoutMs) {
                    std::cout << " [Timeout] SimWorker stuck! Killing process..." << std::endl;
                    kill(pid, SIGKILL);
                    waitpid(pid, &status, 0); // 回收尸体
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (finished) {
                std::ifstream in(outputFile);
                if (in.is_open()) {
                    in >> penalty;
                }
            }
        }
#endif
        return penalty;
    }
};