// Copyright 2020-2021 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MYWINDOW_H
#define MYWINDOW_H

#include <image/image.h>
#include <SDL_opengl.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>
#include <set>
#include "imgui/imgui_impl_sdl.h"
#include "imgui/imgui_impl_opengl2.h"
//#include "imgui/imgui_sdl.cpp"
#include "imgui/imgui_sdl.h"
#include "SDL.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#define GL_GLEXT_PROTOTYPES 1
#include <SDL_opengles2.h>

struct MyCommonEvent {
  bool shiftModifier = false;
  bool ctrlModifier = false;
  bool altModifier = false;
  bool noModifier = false;
};

struct MyMouseEvent : public MyCommonEvent {
  Eigen::Vector2d pos;
  bool leftButton = false;
  bool middleButton = false;
  bool rightButton = false;
  int pressReleaseDurationMs = 0;
  int numClicks = 0;
  void display() const {
    std::cout << pos(0) << " " << pos(1) << " " << leftButton << " "
              << middleButton << " " << rightButton << std::endl;
  }
};

typedef SDL_FingerID FingerId;

struct MyFingerEvent {
  Eigen::Vector2d pos;
  FingerId fingerId;
  int numFingers = 0;
};

struct MyKeyEvent : public MyCommonEvent {
  Sint32 key;
};

struct MyWindowGLData {
  GLuint VAO, VBO_V, VBO_F, VBO_T;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> V, T;
  Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      F;
  GLuint screenTexture, screenShader;
};

class MyWindow {
 public:
  MyWindow(int w, int h, const std::string &windowTitle);

  virtual ~MyWindow();
  ///////////////////
  
  void gui(SDL_Window* window);
  //////////////
  void runLoop();
  SDL_Renderer *getRenderer() const;
  SDL_Window *getWindow() const;
  SDL_GLContext getContext() const;
  const MyWindowGLData &getGLData() const;
  void setMouseEventsSimulationByTouch(bool enable = true);
  int getWidth() const;
  int getHeight() const;
  void setKeyboardEventState(bool enabled);
  void enableKeyboardEvents();
  void disableKeyboardEvents();

  void changekey1value();
  void changekey2value();
  void changekey3value();
  void changekey4value();
  void changesmvalue();
  void changemkvalue();

  void ChangeUI();
  bool key1value();
  bool key2value();
  bool key3value();
  bool key4value();

  bool smvalue();
  bool mkvalue();
  bool ShowUI();

  bool smline = false;
  bool mksk = false;
  bool key1 = false;
  bool key2 = false;
  bool key3 = false;
  bool key4 = false;


 protected:
  virtual bool paintEvent() = 0;
  virtual void mouseEvent(const MyMouseEvent &event){};
  virtual void mouseMoveEvent(const MyMouseEvent &event){};
  virtual void mousePressEvent(const MyMouseEvent &event){};
  virtual void mouseReleaseEvent(const MyMouseEvent &event){};
  virtual void keyEvent(const MyKeyEvent &event){};
  virtual void keyPressEvent(const MyKeyEvent &event){};
  virtual void keyReleaseEvent(const MyKeyEvent &event){};
  virtual void fingerEvent(const MyFingerEvent &event){};
  virtual void fingerMoveEvent(const MyFingerEvent &event){};
  virtual void fingerPressEvent(const MyFingerEvent &event){};
  virtual void fingerReleaseEvent(const MyFingerEvent &event){};

 protected:
  int init(const std::string& windowTitle);
  void initOpenGLBuffers();
  void destroyOpenGLBuffers();
  void openGLDrawScreen(const Imguc &screenImg);
  void destroy();
  virtual bool mainTick();
#ifdef __EMSCRIPTEN__
  static void mainTickEmscripten(void *data);
#endif

  SDL_Window *window = nullptr;
  SDL_Window* window_ui = nullptr;
  SDL_Renderer *renderer = nullptr;
  SDL_Renderer* renderer_ui = nullptr;
  SDL_GLContext context = nullptr;
  SDL_GLContext context_ui;
  const int windowWidth, windowHeight;
  MyKeyEvent lastKeyEvent;
  int lastPressTimestamp = 0;
  bool simulateMouseEventByTouch = true;
  std::set<int> currFingerIds;
  MyWindowGLData glData;
  bool repaint = true;
  /////
  bool show_another_window = true;

  


  
  
};

#endif  // MYWINDOW_H
