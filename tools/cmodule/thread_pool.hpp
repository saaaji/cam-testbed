#pragma once

#include <vector>
#include <thread>
#include <functional>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

class Barrier {
public:
  Barrier(int count) : count(count) {}
  ~Barrier() = default;
  void mark_complete();
  void wait();

private:
  std::mutex mut;
  std::condition_variable cv;
  std::atomic<int> count;
};

class ThreadPool {
public:
  ThreadPool(int num_threads = std::thread::hardware_concurrency());
  ~ThreadPool();
  void enq(std::function<void()> task);

private:
  std::vector<std::thread> workers;
  std::queue<std::function<void()>> tasks;

  std::mutex q_mut; // q mutex
  std::condition_variable q_cv; // queue CV

  bool stop = false;
};