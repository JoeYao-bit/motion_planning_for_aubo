//
// Created by yaozhuo on 2021/9/3.
//

#include "thread_pool.h"

std::mutex mutex_;

ThreadPool::ThreadPool(int number_of_thread) {
    for(int i=0; i<number_of_thread; i++) {
        pool_.emplace_back([this]() { ThreadPool::DoWork(); });
    }
}

ThreadPool::~ThreadPool() {
    allJoin();
}

void ThreadPool::Schedule(const std::function<void()>& work_item) {
    mutex_.lock();
    work_queue_.push_back(work_item);
    mutex_.unlock();
}

bool ThreadPool::allFinish() {
    for(int i=0; i<pool_.size(); i++) {
        if(!pool_[i].joinable()) { return false; }
    }
    std::cout << " all finish " << std::endl;
    return true;
}

void ThreadPool::allJoin() {
    for (std::thread& thread : pool_) {
        if(thread.joinable())
        {
            thread.join();
        }
    }
}

void ThreadPool::allDetach() {
    for (std::thread& thread : pool_) {
        thread.detach();
    }
}

void ThreadPool::DoWork() {
    std::unique_lock<std::mutex> lck(mutex_);
    // wait until start planner is prepared
    cv.wait(lck,[this]{return work_queue_.empty();});
    lck.unlock();
    cv.notify_one();
    while(true) {
        mutex_.lock();
        std::function<void()> work_item;
        if (!work_queue_.empty()) {
            work_item = work_queue_.front();
            work_queue_.pop_front();
        } else {
            mutex_.unlock();
            continue;
        }
        mutex_.unlock();
        work_item();
    }
}