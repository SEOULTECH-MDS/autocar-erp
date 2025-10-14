// Minimal shim for autoware::signal_processing::LowpassFilter1d
// Provides a simple first-order low-pass filter with an interface
// compatible with YabLoc's usage in ground_server.

#ifndef AUTOWARE__SIGNAL_PROCESSING__LOWPASS_FILTER_1D_HPP_
#define AUTOWARE__SIGNAL_PROCESSING__LOWPASS_FILTER_1D_HPP_

#include <optional>

namespace autoware
{
namespace signal_processing
{

class LowpassFilter1d
{
public:
  explicit LowpassFilter1d(double alpha) : alpha_(alpha) {}

  void reset(double value)
  {
    value_ = value;
    has_value_ = true;
  }

  std::optional<double> getValue() const
  {
    if (!has_value_) return std::nullopt;
    return value_;
  }

  double filter(double input)
  {
    if (!has_value_) {
      reset(input);
      return value_;
    }
    value_ = alpha_ * input + (1.0 - alpha_) * value_;
    return value_;
  }

private:
  double alpha_{0.2};
  double value_{0.0};
  bool has_value_{false};
};

}  // namespace signal_processing
}  // namespace autoware

#endif  // AUTOWARE__SIGNAL_PROCESSING__LOWPASS_FILTER_1D_HPP_


