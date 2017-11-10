#ifndef MEAN_FILTER_H
#define MEAN_FILTER_H

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/utility.hpp>

class MeanFilter {
  typedef boost::accumulators::stats<boost::accumulators::tag::rolling_mean>
      AccumulatorStats;
  typedef boost::accumulators::accumulator_set<double, AccumulatorStats>
      RollingMeanAccumulator;
  typedef boost::accumulators::tag::rolling_window RollingWindow;

 public:
  MeanFilter(size_t acc_window_size)
      : accumulator_(RollingWindow::window_size = acc_window_size) {}

  double filter(double value) {
    accumulator_(value);
    return boost::accumulators::rolling_mean(accumulator_);
  }

 private:
  RollingMeanAccumulator accumulator_;
};

#endif  // MEAN_FILTER_H
