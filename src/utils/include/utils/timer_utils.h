/*
 * @Description: 
 * @Autor: 
 * @Date: 2021-12-23 14:41:33
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2021-12-23 14:41:34
 */

#ifndef _TIMER_UTILS_H_
#define _TIMER_UTILS_H_

#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>

class TimerUtils
{
public:
    TimerUtils() : expired_(true), try_to_expire_(false)
    {
    }

    TimerUtils(const TimerUtils &timer)
    {
        expired_ = timer.expired_.load();
        try_to_expire_ = timer.try_to_expire_.load();
    }

    ~TimerUtils()
    {
        stop();
    }

    void start(int interval, std::function<void()> task)
    {
        // is started, do not start again
        if (expired_ == false)
            return;

        // start async timer, launch thread and wait in that thread
        expired_ = false;
        std::thread([this, interval, task]()
                    {
			while (!try_to_expire_)
			{
				// sleep every interval and do the task again and again until times up
				std::this_thread::sleep_for(std::chrono::milliseconds(interval));
				task();
			}

			{
				// timer be stopped, update the condition variable expired and wake main thread
				std::lock_guard<std::mutex> locker(_mutex);
				expired_ = true;
				_expired_cond.notify_one();
			} })
            .detach(); // detach是使主线程不用等待子线程可以继续往下执行，但即使主线程终止了，子线程也不一定终止
    }

    void startOnce(int delay, std::function<void()> task)
    {
        std::thread([delay, task]()
                    {
			std::this_thread::sleep_for(std::chrono::milliseconds(delay));
			task(); })
            .detach();
    }

    void stop()
    {
        // do not stop again
        if (expired_)
            return;

        if (try_to_expire_)
            return;

        // wait until timer
        try_to_expire_ = true; // change this bool value to make timer while loop stop
        {
            std::unique_lock<std::mutex> locker(_mutex);
            _expired_cond.wait(locker, [this]
                               { return expired_ == true; });

            // reset the timer
            if (expired_ == true)
                try_to_expire_ = false;
        }
    }

private:
    std::atomic<bool> expired_;       // timer stopped status
    std::atomic<bool> try_to_expire_; // timer is in stop process
    std::mutex _mutex;
    std::condition_variable _expired_cond;
};

#endif // !_TIMER_H_

/*
void func1()
{
	std::cout << "trigger func1" << std::endl;
}

void func2(int x)
{
	std::cout << "trigger func2, x: " << x << std::endl;
}

int main(int argc, char* argv[])
{
	Timer timer;

	// execute task every timer interval
	std::cout << "--- start period timer ----" << std::endl;
	timer.start(1000, std::bind(func2, 3));
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	timer.stop();
	std::cout << "--- stop period timer ----" << std::endl;

	// execute task once after delay
	std::cout << "--- start one shot timer ----" << std::endl;
	timer.startOnce(1000, func1);
	std::cout << "--- stop one shot timer ----" << std::endl;

	getchar();
	return 0;
}
*/