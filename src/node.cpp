#include <node.hpp>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>

#include <ch.hpp>

namespace Node {

  uavcan_stm32::CanInitHelper<> can;

  uavcan::Node<NodePoolSize>& getNode() {
    static uavcan::Node<NodePoolSize> node(can.driver, uavcan_stm32::SystemClock::instance());
    return node;
  }

  void publishCameraTrigger() {

  }

  void uavcanNodeThread::main() {
    uavcan::uint32_t bitrate = 1000000;
    can.init(bitrate);

    getNode().setName("org.kmti.gmm_controler");
    getNode().setNodeID(10);

    if (getNode().start() < 0) {
      chSysHalt("UAVCAN init fail");
    }

    uavcan::Publisher<uavcan::protocol::debug::KeyValue> kv_pub(getNode());
    kv_pub.init();

    getNode().setModeOperational();

    while(true) {
      if (getNode().spin(uavcan::MonotonicDuration::fromMSec(1000)) < 0) {
        chSysHalt("UAVCAN spin fail");
      }
    }
  }
}

