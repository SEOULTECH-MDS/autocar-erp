// Minimal shim for autoware_utils::StopWatch used in YabLoc
#ifndef AUTOWARE_UTILS__SYSTEM__STOP_WATCH_HPP_
#define AUTOWARE_UTILS__SYSTEM__STOP_WATCH_HPP_

#include <chrono>

namespace autoware_utils
{
class StopWatch
{
public:
  StopWatch() : start_(std::chrono::steady_clock::now()) {}
  double toc() const
  {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now() - start_).count();
  }
  void tic() { start_ = std::chrono::steady_clock::now(); }

private:
  std::chrono::steady_clock::time_point start_;
};
}  // namespace autoware_utils

#endif  // AUTOWARE_UTILS__SYSTEM__STOP_WATCH_HPP_


