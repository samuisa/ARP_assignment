#include <poll.h>
#include <unistd.h>
#include <signal.h>
#include <sys/timerfd.h>
#include <sys/wait.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <vector>
#include <stdexcept>
#include "Pipe.h"
#include "Logger.h"
#include "config.h"
#include "BlackBoard.h"

static volatile bool shutdownFlage = false;
struct WatchedProcess {
    char id;
    std::string name;
    pid_t pid;
    Pipe<char>* pipe;
    int timer_fd; // one-shot timer per process
};

// Forward declarations
void shutdown_all_processes(const std::vector<WatchedProcess>& procs, BlackBoard& blackboard, Logger& logger);
int create_timerfd();
void reset_timer(int fd);
void general_cleanUp();
void heartbeat_received(WatchedProcess& p, Logger& logger);
void handle_signal(int signum) {shutdownFlage = true;}

int main() {
    signal(SIGINT, handle_signal);  // For SIGINT from Ctrl+C (user)

    BlackBoard blackboard;
    Logger logger(SYSTEM_WIDE_LOG);

    // Create pipe objects
    Pipe<char> gameloop_pipe(GAMELOOP_PIPE_WD);
    Pipe<char> keyboard_pipe(KEYBOARD_PIPE_WD);
    Pipe<char> itemspawner_pipe(ITEMSPAWNER_PIPE_WD);
    Pipe<char> master_pipe(MASTER_PIPE_WD);
    Pipe<char> global_timer_pipe(GLOBALTIMER_PIPE_WD);

    // Build list of watched processes
    std::vector<WatchedProcess> procs = {
        {'T', "GlobalTimer", blackboard.getProcessPid(WatchDogProcName::GlobalTimer_Proc), &global_timer_pipe, create_timerfd()},
        {'K', "Keyboard",    blackboard.getProcessPid(WatchDogProcName::Keyboard_Proc),    &keyboard_pipe,    create_timerfd()},
        {'G', "GameLoop",    blackboard.getProcessPid(WatchDogProcName::GameLoop_Proc),    &gameloop_pipe,    create_timerfd()},
        {'S', "ItemSpawner", blackboard.getProcessPid(WatchDogProcName::ItemSpawner_Proc), &itemspawner_pipe, create_timerfd()},
        {'M', "Master",      blackboard.getProcessPid(WatchDogProcName::Master_Proc),      &master_pipe,      create_timerfd()}
    };

    // Initialize all timers
    for (auto& p : procs) reset_timer(p.timer_fd);

    // Build pollfd table
    // struct pollfd {
    //     int fd;        // The file descriptor to monitor
    //     short events;  // The events you're interested in (e.g., POLLIN, POLLOUT)
    //     short revents; // The events that actually occurred
    // };
    std::vector<pollfd> fds;
    for (auto& p : procs) {
        fds.push_back({ p.pipe->get_fd(),  POLLIN, 0 });
        fds.push_back({ p.timer_fd,        POLLIN, 0 });
    }

    // Log all running processes
    for (auto& p : procs) logger.log(p.name + " is running...", p.pid, Logger::LogLevel::INFO);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    logger.log("Watchdog is monitoring all processes.", getpid(), Logger::LogLevel::INFO);

    while (true) {
        int ret = poll(fds.data(), fds.size(), -1);
        if (ret < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error("poll failed");
        }

        for (size_t i = 0; i < procs.size(); ++i) {
            auto& p = procs[i];
            pollfd& pipefd  = fds[2*i];
            pollfd& timerfd = fds[2*i+1];

            // ----------------- PIPE HEARTBEAT -----------------
            if (pipefd.revents & POLLIN) {
                char* msg = p.pipe->receive_data();
                if (msg) {
                    if (*msg == p.id) {
                        heartbeat_received(p, logger);
                    } else if (*msg == 'Q') {
                        logger.log("'Q' received from: " + p.name, p.pid, Logger::LogLevel::WARNING);
                        shutdown_all_processes(procs, blackboard, logger);
                        return EXIT_SUCCESS;
                    } else {
                        logger.log("Unexpected message from " + p.name + ": " + *msg, p.pid, Logger::LogLevel::WARNING);
                    }
                } else {
                    logger.log("No heartbeat received yet from " + p.name, p.pid, Logger::LogLevel::LOG);
                }
            }

            // ----------------- TIMER EXPIRED -----------------
            if (timerfd.revents & POLLIN) {
                logger.log("Timeout detected: " + p.name, p.pid, Logger::LogLevel::ERROR);
                shutdown_all_processes(procs, blackboard, logger);
                return EXIT_FAILURE;
            }
        }

        if (shutdownFlage) { // check the flag
            logger.log("Received SIGINT (Ctrl+C). Shutting down...", getpid(), Logger::LogLevel::WARNING);
            shutdown_all_processes(procs, blackboard, logger);
            return EXIT_SUCCESS;
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return EXIT_SUCCESS;
}

void heartbeat_received(WatchedProcess& p, Logger& logger) {
    reset_timer(p.timer_fd); // reset only this process’s timer
    logger.log("Heartbeat received: " + p.name, p.pid, Logger::LogLevel::LOG, false);
}

void shutdown_all_processes(const std::vector<WatchedProcess>& procs, BlackBoard& blackboard, Logger& logger) {
    logger.log("Watchdog initiating shutdown of all processes", getpid(), Logger::LogLevel::INFO);

    if (!shutdownFlage) {  // if CTRL + C pressed don't send SIGTERM, terminate immediately
        // Send SIGTERM first
        for (const auto& p : procs) {
            if (p.pid > 0) kill(p.pid, SIGTERM);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Force kill any remaining processes
    for (const auto& p : procs) {
        if (p.pid <= 0) 
            continue;  // Skip invalid PIDs
        if (kill(p.pid, 0) == 0) { // check if the process is still running
            logger.log("Watchdog Shutting down: " + p.name + " PID=" + std::to_string(p.pid), getpid(), Logger::LogLevel::WARNING);
            kill(p.pid, SIGKILL);  // Force kill the process
        } else {
            logger.log("Process already exited: " + p.name + " PID=" + std::to_string(p.pid), getpid(), Logger::LogLevel::INFO);
        }
    }

    // Clean up shared resources
    blackboard.clean_up();
    general_cleanUp();

    logger.log("All processes terminated by Watchdog", getpid(), Logger::LogLevel::INFO);
}

int create_timerfd() {
    int fd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    if (fd < 0) throw std::runtime_error("timerfd_create failed");
    return fd;
}

void reset_timer(int fd) {
    itimerspec ts{};
    ts.it_value.tv_sec = WATCHDOG_TIMEOUT_SECONDS;
    ts.it_value.tv_nsec = 0;
    ts.it_interval.tv_sec = 0;   // one-shot timer
    ts.it_interval.tv_nsec = 0;
    timerfd_settime(fd, 0, &ts, nullptr);
}

void general_cleanUp() {
    shm_unlink(SHM_NAME);   
    sem_unlink(SPAWN_SEM_NAME);  
    unlink(KEYBOARD_Data_PIPE);
    unlink(GLOBALTIMER_PIPE_WD);
    unlink(GAMELOOP_PIPE_WD);
    unlink(MASTER_PIPE_WD);
    unlink(KEYBOARD_PIPE_WD);
    unlink(ITEMSPAWNER_PIPE_WD);
}
