#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

#define BETA 0.75
#define MAX_DELAY 200
// One tick in Sprout algo
#define TICK_SIZE_MS 20
// Maximum time for ack to return to sender
#define RECV_DELAY_MS 200

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( true || debug ), last_acked_sequence_number_(0),
  window_size_(1), window_acks_(0),
  last_update_ms_(timestamp_ms() + RECV_DELAY_MS),
  packets_recv_(), queue_size_estimate_(0), lambda_distr_(),
  lambda_support_()
{
  int blah = 50;
  for (int i = 0; i < blah; i++) {
    double support = i * 1000. / blah;
    lambda_support_.push_back(support);
    lambda_distr_[support] = 1. / blah;
  }
}

/* Get current window size, in datagrams */
unsigned int Controller::window_size()
{
  uint64_t current_time = timestamp_ms();
  while (current_time >= last_update_ms_ + TICK_SIZE_MS) {
    vector<uint64_t> remaining;
    int packets_in_update_window = 0;
    for (uint64_t ack_timestamp: packets_recv_) {
      if (ack_timestamp <= last_update_ms_ -
          RECV_DELAY_MS) {
        packets_in_update_window++;
      } else {
        remaining.push_back(ack_timestamp);
      }
    }

    lambda_distr_ = brownian(lambda_distr_);
    update_distr(packets_in_update_window);
    packets_recv_ = remaining;
    last_update_ms_ += TICK_SIZE_MS;

    window_size_ = forecast();
    cout << window_size_ << endl;
  }

  /* Default: fixed window size of 100 outstanding datagrams */
  // unsigned int the_window_size = 50;
  unsigned int the_window_size = window_size_;

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size << endl;
  }

  return the_window_size;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp,
                                    /* in milliseconds */
				    const bool after_timeout
				    /* datagram was sent because of a timeout */ )
{
  queue_size_estimate_++;
  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << " (timeout = " << after_timeout << ")\n";
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  packets_recv_.push_back(recv_timestamp_acked);
  queue_size_estimate_--;
  window_acks_ += sequence_number_acked - last_acked_sequence_number_;
  last_acked_sequence_number_ = sequence_number_acked;
  //if (window_acks_ >= window_size_) {
  //  window_size_++;
  //  window_acks_ = 0;
  //}

  //if ((timestamp_ack_received - send_timestamp_acked) <= MAX_DELAY / 4) {
  //  window_acks_ += 10;
  //}

  //if ((timestamp_ack_received - send_timestamp_acked) >= MAX_DELAY) {
  //  window_acks_ = 0;
  //  window_size_ *= BETA;
  //  if (window_size_ == 0) {
  //    window_size_ = 1;
  //  }
  //}

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  return 1000; /* timeout of one second */
}

void Controller::update_distr(int recv_packets) {
  double mean = 0;
  for (auto &supports: lambda_distr_) {
    cout << supports.first << ": " << supports.second << endl;
    mean += supports.first * supports.second;
  }
  cout << "Mean: " << mean << endl;
  cout << endl << endl;

  double normalization_factor = 0.;
  for (auto &supports: lambda_distr_) {
    Poisson p(supports.first * TICK_SIZE_MS / 1000.);
    supports.second *= (p.pdf(recv_packets) + 0.00001);
    normalization_factor += supports.second;
  }

  for (auto &supports: lambda_distr_) {
    supports.second /= normalization_factor;
  }
}

unordered_map<double, double> Controller::brownian(
    const unordered_map<double, double> &lambda_distr
    ) {
  unordered_map<double, double> updated(lambda_distr);
  for (auto const &supports: lambda_distr) {
    double weight = supports.second;
    NormalDistribution normal(supports.first, 200);
    for (auto &supports2: updated) {
      supports2.second += weight * normal.pdf(supports2.second);
    }
  }

  double normalization_factor = 0.;
  for (auto &supports: updated) {
    normalization_factor += supports.second;
  }

  for (auto &supports: updated) {
    supports.second /= normalization_factor;
  }
  return updated;
}

int Controller::forecast() {
  unordered_map<double, double> lambda_d(lambda_distr_);
  unordered_map<double, double> cum_lambda_d;
  for (int i = 0; i < MAX_DELAY / TICK_SIZE_MS; i++) {
    lambda_d = brownian(lambda_d);
    for (const auto &lambda : lambda_d) {
      cum_lambda_d[lambda.first] +=
        lambda.second / (MAX_DELAY / TICK_SIZE_MS);
    }
  }

  double cdf = 0;
  for (double lambda: lambda_support_) {
    cdf += cum_lambda_d[lambda];
    if (cdf > 0.05) {
      return lambda * MAX_DELAY / 1000.;
    }
  }

  int x = *(int *)0;
  x++;
  return 0;
}
