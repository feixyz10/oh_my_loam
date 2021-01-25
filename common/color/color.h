#pragma once

namespace common {

struct Color {
  unsigned char r, g, b;
  Color(unsigned char r, unsigned char g, unsigned char b) : r(r), g(g), b(b) {}
};

#define BLACK Color(0, 0, 0)
#define WHITE Color(255, 255, 255)
#define RED Color(255, 0, 0)
#define GREEN Color(0, 255, 0)
#define BLUE Color(0, 0, 255)
#define DRAK_GRAY Color(50, 50, 50)
#define GRAY Color(128, 128, 128)
#define CYAN Color(0, 255, 255)
#define PURPLE Color(160, 32, 240)
#define VIOLET Color(238, 130, 238)
#define ORANGE Color(255, 97, 0)
#define PINK Color(255, 182, 193)
#define YELLOW Color(255, 255, 0)

}  // namespace common