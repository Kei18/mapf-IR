#include "../include/ofApp.hpp"
#include "../include/params.hpp"


ofApp::ofApp(MAPFPlan* _P): P(_P) {
  flg_autoplay = true;
  flg_loop = true;
  flg_goal = true;
  flg_font = false;
  flg_line = true;
  flg_focus = false;
}

void ofApp::setup() {
  int map_w = P->G->getWidth();
  int map_h = P->G->getHeight();
  int window_max_w = BufferSize::default_screen_width
    - BufferSize::screen_x_buffer * 2 - BufferSize::window_x_buffer * 2;
  int window_max_h = BufferSize::default_screen_height
    - BufferSize::window_y_top_buffer - BufferSize::window_y_bottom_buffer;
  scale = std::min(window_max_w / map_w, window_max_h / map_h) + 1;
  int w = map_w * scale + 2 * BufferSize::window_x_buffer;
  int h = map_h * scale
    + BufferSize::window_y_top_buffer + BufferSize::window_y_bottom_buffer;

  agent_rad = std::max((float)scale/std::sqrt(2)/2, 3.0);
  goal_rad  = std::max((float)scale/4.0, 2.0);
  font_size = std::max(scale/8, 6);

  ofSetWindowShape(w, h);
  ofBackground(Color::bg);
  ofSetCircleResolution(32);
  ofSetFrameRate(30);

  std::string font_name = "MuseoModerno-VariableFont_wght.ttf";
  font.load(font_name, font_size);
  font_info.load(font_name, 10);

  gui.setup();
  gui.add(timestep_slider.setup("time step", 0, 0, P->makespan));
  gui.add(speed_slider.setup("speed", 0.1, 0, 1));
  gui.add(agent_slider.setup("agent", 0, 0, P->num_agents-1));

  printKeys();
}

void ofApp::update() {
  if (!flg_autoplay) return;

  float t;
  t = timestep_slider + speed_slider;
  if (t <= P->makespan) {
    timestep_slider = t;
  } else {
    timestep_slider = 0;
    if (flg_loop) {
      timestep_slider = 0;
    } else {
      timestep_slider = P->makespan;
    }
  }
}

void ofApp::draw() {
  // draw nodes
  ofSetLineWidth(1);
  ofFill();
  for (int x = 0; x < P->G->getWidth(); ++x) {
    for (int y = 0; y < P->G->getHeight(); ++y) {
      if (!P->G->existNode(x, y)) continue;
      ofSetColor(Color::node);
      int x_draw = x*scale-scale/2+0.5
        + BufferSize::window_x_buffer + scale/2;
      int y_draw = y*scale-scale/2+0.5
        + BufferSize::window_y_top_buffer + scale/2;
      ofDrawRectangle(x_draw, y_draw, scale-0.5, scale-0.5);
      if (flg_font) {
        ofSetColor(Color::font);
        font.drawString(std::to_string(y*P->G->getWidth()+x),
                        x_draw + 1, y_draw + font_size + 1);
      }
    }
  }

  // draw goals
  if (flg_goal) {
    for (int i = 0; i < P->num_agents; ++i) {
      if (flg_focus && i != agent_slider) continue;
      ofSetColor(Color::agents[i % Color::agents.size()]);
      Node* g = P->config_g[i];
      Pos pos1 = g->pos * scale;
      int x = pos1.x + BufferSize::window_x_buffer + scale/2;
      int y = pos1.y + BufferSize::window_y_top_buffer + scale/2;
      ofDrawRectangle(x - goal_rad/2, y - goal_rad/2, goal_rad, goal_rad);
    }
  }

  // draw agents
  for (int i = 0; i < P->num_agents; ++i) {
    if (flg_focus && i != agent_slider) continue;
    ofSetColor(Color::agents[i % Color::agents.size()]);
    int t1 = (int)timestep_slider;
    int t2 = t1 + 1;

    // agent position
    Node* v = P->transitions[t1][i];
    Pos pos1 = v->pos;
    float x = pos1.x;
    float y = pos1.y;

    if (t2 <= P->makespan) {
      Pos pos2 = P->transitions[t2][i]->pos;
      x += (pos2.x - x) * (timestep_slider - t1);
      y += (pos2.y - y) * (timestep_slider - t1);
    }
    x *= scale;
    y *= scale;
    x += BufferSize::window_x_buffer + scale/2;
    y += BufferSize::window_y_top_buffer + scale/2;

    ofDrawCircle(x, y, agent_rad);

    // goal
    if (flg_line) {
      Pos pos3 = P->config_g[i]->pos * scale;
      ofDrawLine(pos3.x + BufferSize::window_x_buffer + scale/2,
                 pos3.y + BufferSize::window_y_top_buffer + scale/2, x, y);
    }

    // agent at goal
    if (v == P->config_g[i]) {
      ofSetColor(255,255,255);
      ofDrawCircle(x, y, agent_rad-2);
    }

    // id
    if (flg_font) {
      ofSetColor(Color::font);
      font.drawString(std::to_string(i), x-font_size/2, y+font_size/2);
    }
  }

  // info
  ofSetColor(Color::font_info);
  int x = 220;
  int y = 5;
  font_info.drawString("solved by "
                       + P->solver
                       + (P->solved ? ", success" : ", failed"),
                       x, y+=15);
  font_info.drawString("agents: "
                       + std::to_string(P->num_agents)
                       + ", map: " + P->G->getMapFileName(),
                       x, y+=15);
  font_info.drawString("comp_time: "
                       + std::to_string(P->comp_time) + " ms",
                       x, y+=15);
  font_info.drawString("soc: " + std::to_string(P->soc)
                       + ", makespan: " + std::to_string(P->makespan),
                       x, y+=15);

  gui.draw();
}

void ofApp::keyPressed(int key) {
  if (key == 'r') timestep_slider = 0;  // reset
  if (key == 'p') flg_autoplay = !flg_autoplay;
  if (key == 'l') flg_loop = !flg_loop;
  if (key == 'v') flg_line = !flg_line;
  if (key == 'a') flg_focus = !flg_focus;
  if (key == 'f') {
    flg_font = !flg_font;
    flg_font &= (scale - font_size > 6);
  }

  float t;
  if (key == OF_KEY_RIGHT) {
    t = timestep_slider + speed_slider;
    timestep_slider = std::min((float)P->makespan, t);
  }
  if (key == OF_KEY_LEFT) {
    t = timestep_slider - speed_slider;
    timestep_slider = std::max((float)0, t);
  }
  if (key == OF_KEY_UP) {
    t = speed_slider + 0.001;
    speed_slider = std::min(t, (float)speed_slider.getMax());
  }
  if (key == OF_KEY_DOWN) {
    t = speed_slider - 0.001;
    speed_slider = std::max(t, (float)speed_slider.getMin());
  }
  if (key == '+') {
    agent_slider = std::min(agent_slider+1, P->num_agents);
  }
  if (key == '-') {
    agent_slider = std::max(agent_slider-1, 0);
  }
}

void ofApp::dragEvent(ofDragInfo dragInfo) {}
void ofApp::keyReleased(int key) {}
void ofApp::mouseMoved(int x, int y) {}
void ofApp::mouseDragged(int x, int y, int button) {}
void ofApp::mousePressed(int x, int y, int button) {}
void ofApp::mouseReleased(int x, int y, int button) {}
void ofApp::mouseEntered(int x, int y) {}
void ofApp::mouseExited(int x, int y) {}
void ofApp::windowResized(int w, int h) {}
void ofApp::gotMessage(ofMessage msg) {}

void ofApp::printKeys()
{
  std::cout << "keys for visualizer" << std::endl;
  std::cout << "- p : play or pause" << std::endl;
  std::cout << "- l : loop or not" << std::endl;
  std::cout << "- r : reset" << std::endl;
  std::cout << "- v : show virtual line to goals" << std::endl;
  std::cout << "- f : show agent & node id" << std::endl;
  std::cout << "- right : progress" << std::endl;
  std::cout << "- left  : back" << std::endl;
  std::cout << "- up    : speed up" << std::endl;
  std::cout << "- down  : speed down" << std::endl;
  std::cout << "- a : show single agent" << std::endl;
  std::cout << "- + : increment single agent id" << std::endl;
  std::cout << "- - : decrement single agent id" << std::endl;
}
