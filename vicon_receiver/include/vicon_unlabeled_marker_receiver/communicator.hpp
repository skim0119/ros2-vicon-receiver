#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "DataStreamClient.h"
#include "rclcpp/rclcpp.hpp"
#include "vicon_unlabeled_marker_receiver/publisher.hpp"
#include <boost/thread.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <unistd.h>

using namespace std;

namespace ViconReceiver {
namespace UnlabeledMarker {
// Main Node class
class Communicator : public rclcpp::Node {
private:
  ViconDataStreamSDK::CPP::Client vicon_client;
  string hostname;
  unsigned int buffer_size;
  string ns_name;
  map<string, Publisher> pub_map;
  boost::mutex mutex;

public:
  Communicator();

  // Initialises the connection to the DataStream server
  bool connect();

  // Stops the current connection to a DataStream server (if any).
  bool disconnect();

  // Main loop that request frames from the currently connected DataStream
  // server and send the received segment data to the Publisher class.
  void get_frame();

  // functions to create a segment publisher in a new thread
  void create_publisher(const string subject_name, const string segment_name);
  void create_publisher_thread(const string subject_name,
                               const string segment_name);
};

} // namespace UnlabeledMarker
} // namespace ViconReceiver

#endif // COMMUNICATOR_HPP