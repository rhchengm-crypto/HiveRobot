#pragma once
#include <stdint.h>
class SpeechOrder {
 public:
  constexpr void reset(){next_=0;feed_=0;}
  constexpr uint8_t take(uint32_t feed) {
    if(feed_!=feed){next_=0;feed_=feed;}
    const uint8_t selected=next_;next_=(next_+1)%5;return selected;
  }
 private:
  uint32_t feed_=0;
  uint8_t next_=0;
};
constexpr bool testSpeechOrder() {
  SpeechOrder order;
  for(unsigned i=0;i<12;++i)if(order.take(1)!=i%5)return false;
  if(order.take(2)!=0 || order.take(2)!=1)return false;
  order.reset();return order.take(2)==0;
}
static_assert(testSpeechOrder(),"Speech order, wrap, new-feed and context reset failed");
