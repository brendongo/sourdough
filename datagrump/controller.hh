#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <map>

/* Congestion controller interface */
class NormalDistribution {
  private:
    double std_;
    //std::map<double, double> probs_;
    //std::vector<double> probs_;
    double probs_[200];

  public:
    NormalDistribution(double std) : std_(std), probs_() {
      int blah = 200;
      for (int i = 0; i < blah; i++) {
        double support = i * 800. / blah;
        probs_[i] = 
            1. / sqrt(2 * M_PI * std_ * std_) * exp(-pow(support, 2) / (2 * std_ * std_));
        //std::cout << "Normal(" << support << ") = " << probs_[i] << std::endl;
        //probs_.push_back(support);
        //probs_[support] = 0;
            //1. / sqrt(2 * M_PI * std_ * std_) * exp(-pow(support, 2) / (2 * std_ * std_));
      }
    }

    double pdf(double x) {
      return probs_[int(std::abs(x) * 200 / 1000.)];
      //return probs_[int(x * 200. / 1000.)];
      //return probs_.at(std::abs(x));
    }
};

class Controller
{
private:
  bool debug_; /* Enables debugging output */

  /* Add member variables here */
  uint64_t last_acked_sequence_number_;
  int window_size_;
  unsigned int window_acks_;

  // Timestamp when we last updated lambda distr
  uint64_t last_update_ms_;
  // all the packets received up to RECV_DELAY_MS +
  // TICK_SIZE_MS ago (list of recv timestamps)
  std::vector<uint64_t> packets_recv_;

  // Upper bound of packets currently in the router buffer
  int queue_size_estimate_;

  std::unordered_map<double, double> lambda_distr_;
  std::vector<double> lambda_support_;

  NormalDistribution gaussian_;

public:
  /* Public interface for the congestion controller */
  /* You can change these if you prefer, but will need to change
     the call site as well (in sender.cc) */

  /* Default constructor */
  Controller( const bool debug );

  /* Get current window size, in datagrams */
  unsigned int window_size();

  /* A datagram was sent */
  void datagram_was_sent( const uint64_t sequence_number,
			  const uint64_t send_timestamp,
			  const bool after_timeout );

  /* An ack was received */
  void ack_received( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );

  /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms();

  void update_distr(int);
  std::unordered_map<double, double> brownian(const std::unordered_map<double, double> &);
  int forecast();
};

class Poisson {
  private:
    double lambda_;

    int factorial(int x) {
      double result = 1;
      for (int i = 1; i <= x; i++) {
        result *= i;
      }
      return result;
    }

  public:
    Poisson(double lambda) : lambda_(lambda) {};
    double pdf(int x) {
      return pow(lambda_, x) * exp(-1 * lambda_) / factorial(x);
    }
};

class NegativeExponential {
  private:
    double lambda_;

  public:
    NegativeExponential(double lambda) : lambda_(lambda) {};
    double pdf(double x) {
      return lambda_ * exp(-1 * lambda_ * x);
    }
};
#endif
