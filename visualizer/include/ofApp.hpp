#pragma once
#include "ofMain.h"
#include "ofxGui.h"
#include "mapfplan.hpp"

class ofApp : public ofBaseApp {
  MAPFPlan* P;

  // params
  int scale;
  float agent_rad;
  float goal_rad;
  int font_size;

  // gui
  ofxFloatSlider timestep_slider;
  ofxFloatSlider speed_slider;
  ofxPanel gui;

  // flg
  bool flg_autoplay;
  bool flg_loop;
  bool flg_goal;
  bool flg_font;
  bool flg_line;

  // visual
  ofTrueTypeFont font;
  ofTrueTypeFont font_info;

public:
  ofApp(MAPFPlan* _P);
  ~ofApp() {};
  void setup();
  void update();
  void draw();
  void keyPressed(int key);
  void keyReleased(int key);
  void mouseMoved(int x, int y );
  void mouseDragged(int x, int y, int button);
  void mousePressed(int x, int y, int button);
  void mouseReleased(int x, int y, int button);
  void mouseEntered(int x, int y);
  void mouseExited(int x, int y);
  void windowResized(int w, int h);
  void dragEvent(ofDragInfo dragInfo);
  void gotMessage(ofMessage msg);
  void printKeys();
};
