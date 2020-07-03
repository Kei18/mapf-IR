#pragma once
#include "ofMain.h"

// color scheme
namespace Color {
  static const ofColor bg = ofColor(0, 0, 0);
  static const ofColor node = ofColor(255, 255, 255);
  static const ofColor font = ofColor(100, 100, 100);
  static const ofColor font_info = ofColor(255, 255, 255);
  static const ofColor edge = ofColor(200, 200, 200);
  static const std::vector<ofColor> agents =
    {
     ofColor(233, 30, 99), ofColor(33, 150, 243),
     ofColor(76, 175, 80), ofColor(255, 152, 0),
     ofColor(0, 188, 212),  ofColor(156, 39, 176),
     ofColor(121, 85, 72), ofColor(255, 187, 59),
     ofColor(244, 67, 54), ofColor(96, 125, 139),
     ofColor(0, 150, 136), ofColor(63, 81, 181)
    };
}

// width, height
namespace BufferSize {
  static const int default_screen_width  = 1280;
  static const int default_screen_height = 600;
  static const int screen_x_buffer = 50;
  static const int screen_y_buffer = 75;
  static const int window_x_buffer = 25;
  static const int window_y_top_buffer = 75;
  static const int window_y_bottom_buffer = 25;
}
