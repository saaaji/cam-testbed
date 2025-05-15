#include "thread_pool.hpp"

#include <iostream>

void Barrier::mark_complete() {
  if (--count == 0) {
    std::unique_lock<std::mutex> lock(mut);
    cv.notify_one();
  }
}

void Barrier::wait() {
  std::unique_lock<std::mutex> lock(mut);
  cv.wait(lock, [this]{
    return count == 0;
  });
}

ThreadPool::ThreadPool(int num_threads) {
  for (int i = 0; i < num_threads; i++) {
    workers.emplace_back([this]() {
      while (true) {
        std::function<void()> task;

        {
          std::unique_lock<std::mutex> lock(q_mut);

          q_cv.wait(lock, [this]() {
            return stop || !tasks.empty();
          });

          if (stop && tasks.empty()) {
            return;
          }

          task = std::move(tasks.front());
          tasks.pop();
        }

        task();
      }
    });
  }
}

ThreadPool::~ThreadPool() {
  {
    std::unique_lock<std::mutex> lock(q_mut);
    stop = true;
  }

  q_cv.notify_all();
  for (auto& worker : workers) {
    worker.join();
  }
}

void ThreadPool::enq(std::function<void()> task) {
  std::unique_lock<std::mutex> lock(q_mut);
  tasks.push(std::move(task));
  q_cv.notify_one();
}