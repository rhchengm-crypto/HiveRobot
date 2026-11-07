#pragma once
#include <stdint.h>
struct TouchPoint { int16_t x=0, y=0; };
struct TouchCalibration {
  TouchPoint raw[3];
  constexpr bool valid() const {
    for (const auto& p : raw) if (p.x<50 || p.x>4050 || p.y<50 || p.y>4050) return false;
    const int32_t det = determinant();
    return det > 250000 || det < -250000;
  }
  constexpr int32_t determinant() const {
    return int32_t(raw[1].x-raw[0].x)*(raw[2].y-raw[0].y)
         - int32_t(raw[1].y-raw[0].y)*(raw[2].x-raw[0].x);
  }
  constexpr bool map(uint16_t x, uint16_t y, TouchPoint& screen) const {
    if (!valid()) return false;
    const float dx=int32_t(x)-raw[0].x, dy=int32_t(y)-raw[0].y;
    const float u=(dx*(raw[2].y-raw[0].y)-dy*(raw[2].x-raw[0].x))/determinant();
    const float v=((raw[1].x-raw[0].x)*dy-(raw[1].y-raw[0].y)*dx)/determinant();
    const float sx=35+410*u, sy=65+220*v;
    if (sx < -40 || sx > 519 || sy < -40 || sy > 359) return false;
    screen.x = sx<0?0:sx>479?479:int16_t(sx+0.5f);
    screen.y = sy<0?0:sy>319?319:int16_t(sy+0.5f);
    return true;
  }
};
constexpr int touchPageAt(int16_t x,int16_t y) {
  if(y<36 || y>=72)return -1;
  for(int page=0;page<4;++page)if(x>=12+116*page && x<120+116*page)return page;
  return -1;
}
constexpr bool touchWeatherRefresh(uint8_t page,int16_t x,int16_t y) {
  return page==1 && x>=350 && x<468 && y>=280 && y<312;
}
static_assert(touchPageAt(12,36)==0 && touchPageAt(119,71)==0 && touchPageAt(120,50)==-1
  && touchPageAt(128,36)==1 && touchPageAt(235,71)==1 && touchPageAt(236,50)==-1
  && touchPageAt(244,36)==2 && touchPageAt(351,71)==2 && touchPageAt(352,50)==-1
  && touchPageAt(360,36)==3 && touchPageAt(467,71)==3 && touchPageAt(468,50)==-1
  && touchPageAt(400,35)==-1 && touchPageAt(400,72)==-1,"Four-page tab bounds failed");
static_assert(touchWeatherRefresh(1,350,280) && touchWeatherRefresh(1,467,311)
  && !touchWeatherRefresh(0,350,280) && !touchWeatherRefresh(2,350,280)
  && !touchWeatherRefresh(1,349,280) && !touchWeatherRefresh(1,468,280)
  && !touchWeatherRefresh(1,350,279) && !touchWeatherRefresh(1,350,312),"Weather refresh touch bounds failed");
constexpr bool testTouchGeometry() {
  TouchCalibration c{{{300,500},{3800,500},{300,3600}}}; TouchPoint p;
  if (!c.map(300,500,p) || p.x!=35 || p.y!=65) return false;
  if (!c.map(3800,3600,p) || p.x!=445 || p.y!=285) return false;
  if (!c.map(2050,2050,p) || p.x!=240 || p.y!=175) return false;
  TouchCalibration swapped{{{3700,350},{3700,3700},{400,350}}};
  if (!swapped.map(2050,2025,p) || p.x!=240 || p.y!=175) return false;
  TouchCalibration bad{{{100,100},{200,200},{300,300}}};
  if (bad.valid() || bad.map(200,200,p)) return false;
  return touchPageAt(12,36)==0 && touchPageAt(119,71)==0 && touchPageAt(120,50)==-1
      && touchPageAt(128,36)==1 && touchPageAt(351,71)==2 && touchPageAt(360,50)==3
      && touchPageAt(468,50)==-1 && touchPageAt(100,72)==-1 && touchPageAt(100,35)==-1;
}
static_assert(testTouchGeometry(), "Landscape affine touch axes/edges and navigation hitbox tests failed");

constexpr int touchBulbAt(uint8_t page, int16_t x, int16_t y) {
  if (page!=0 || y<82 || y>=270) return -1;
  for (uint8_t i=0; i<3; ++i) {
    const int left=12+i*156;
    if (x>=left && x<left+144) return i;
  }
  return -1;
}
constexpr bool testBulbCardHitboxes() {
  for (uint8_t i=0;i<3;++i) {
    const int left=12+i*156;
    if (touchBulbAt(0,left,82)!=i || touchBulbAt(0,left+143,269)!=i) return false;
    if (touchBulbAt(0,left+144,120)!=-1 || touchBulbAt(0,left,81)!=-1
        || touchBulbAt(0,left,270)!=-1 || touchBulbAt(1,left,120)!=-1
        || touchBulbAt(2,left,120)!=-1) return false;
  }
  return touchBulbAt(0,11,120)==-1 && touchBulbAt(0,200,50)==-1;
}
static_assert(testBulbCardHitboxes(), "Bulb touch targets/gaps/edges and inactive-page tests failed");
