//
// Created by yaozhuo on 2021/9/3.
//

#ifndef _THREAD_POOL_H_
#define _THREAD_POOL_H_

#include <iostream>
#include <thread>
#include <functional>
#include <vector>
#include <deque>
#include <condition_variable>

struct ThreadPool {
public:
    explicit ThreadPool(int number_of_thread = 4);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    void Schedule(const std::function<void()>& work_item);

    std::deque<std::function<void()>> work_queue_;

    void DoWork();

    bool allFinish();

    void allJoin();

    void allDetach();

    std::vector<std::thread> pool_;
    std::condition_variable cv;
};

#endif //FREENAV_THREAD_POOL_H
