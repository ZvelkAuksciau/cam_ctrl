#ifndef NODE_HPP
#define NODE_HPP

#include <uavcan/uavcan.hpp>
#include <ch.hpp>

namespace Node {

  constexpr unsigned NodePoolSize = 400;
  uavcan::Node<NodePoolSize>& getNode();

  class uavcanNodeThread : public chibios_rt::BaseStaticThread<4000> {
    public:
      void main();
  };

}

#endif

