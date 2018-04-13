#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

#define BETA 0.75
#define MAX_DELAY 200

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), last_acked_sequence_number_(0), window_size_(1),
    window_acks_(0)
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size()
{
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
  window_acks_ += sequence_number_acked - last_acked_sequence_number_;
  last_acked_sequence_number_ = sequence_number_acked;
  if (window_acks_ >= window_size_) {
    window_size_++;
    window_acks_ = 0;
  }

  if ((timestamp_ack_received - send_timestamp_acked) <= MAX_DELAY / 4) {
    window_acks_ += 10;
  }

  if ((timestamp_ack_received - send_timestamp_acked) >= MAX_DELAY) {
    window_acks_ = 0;
    window_size_ *= BETA;
    if (window_size_ == 0) {
      window_size_ = 1;
    }
  }

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
