#ifndef PERFORMANCE_ANALYZER_H_
#define PERFORMANCE_ANALYZER_H_
#include <functional>

/**
 * RMSEEvaluator class implements RMSE (Root mean square error) for a given set
 * of estimates and ground truth data of a given type T. The type T is expected
 * to provide -, * and sqrt operations. Otherwise, two operators should be
 * passed to the constructor: 

       RMSEEvaluator(std::function<T(T&, const T&, const T&)> const& add_diff, 
       std::function<T(const T&, long long)> const& mean)
 * 
 * The implementation aimed to minimize memory consumption in order to handle a
 * large number of estimates.
 */
template <typename T>
class RMSEEvaluator {
  // Total number of estimates so far
  long long count_ = 0;

  // Sum of the square differences netween estimates and their ground truths
  T sum_;

  // Operator add square difference between an estimate and its ground truth
  std::function<T(T&, const T&, const T&)> add_diff_;

  // Operator to compute the square root means from the sum and estimate counts
  std::function<T(const T&, long long)> mean_;

 public:
  /**
   * Constructor
   */
  RMSEEvaluator() {}

  /**
   * Constructor
   * @param add_diff the operator to add the square difference difference, it
   * takes 3 arguments:
   *        the result where the square difference is to be added into, the
   * estimate, and the ground truth.
   * @param mean_ the operator to compute the mean from the sum of the sequre
   * differences,
   *        it takes 2 arguments: the sum of the square difference, and the
   * total data count.
   */
  RMSEEvaluator(std::function<T(T&, const T&, const T&)> const& add_diff,
                std::function<T(const T&, long long)> const& mean) {
    add_diff_ = add_diff;
    mean_ = mean;
  }

  /**
   * Destructor
   */
  ~RMSEEvaluator() {}

  /**
   * Reset the evaluator so it can be restarted
   */
  void Reset() { count_ = 0; }

  /**
   * Add an estimate and ground truth
   * @param estimate the estimate
   * @param ground_truth the ground truth
   */
  void Add(T estimate, T ground_truth) {
    if (add_diff_) {
      add_diff_(sum_, estimate, ground_truth);
    } else {
      T diff = estimate - ground_truth;
      if (count_ > 0) {
        sum_ += diff * diff;
      } else {
        sum_ = diff * diff;
      }
    }

    count_++;
  }

  /**
   * Evaluate the current data, and return the RMSE result
   */
  T Evaluate() {
    if (mean_) {
      return mean_(sum_, count_);
    } else {
      T mean = sum_ / count_;
      return mean.sqrt();
    }
  }
};

#endif