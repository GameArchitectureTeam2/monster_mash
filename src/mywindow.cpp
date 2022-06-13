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

#include "mywindow.h"

#include <miscutils/opengltools.h>

#include <iostream>

#include "macros.h"

using namespace std;
using namespace Eigen;

MyWindow::MyWindow(int w, int h, const std::string& windowTitle)
    : windowWidth(w), windowHeight(h) {
    init(windowTitle);
}

MyWindow::~MyWindow() { destroy(); }

int MyWindow::init(const std::string& windowTitle) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        cerr << "SDL_Init Error: " << SDL_GetError() << endl;
        return 1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    window = SDL_CreateWindow(windowTitle.c_str(), SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED, windowWidth, windowHeight,
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    context = SDL_GL_CreateContext(window);
    if (SDL_GL_SetSwapInterval(-1) == -1) {
        SDL_GL_SetSwapInterval(1);
    }
    cout << "GL version: " << glGetString(GL_VERSION) << endl;
    if (window == nullptr) {
        cerr << "SDL_CreateWindow error: " << SDL_GetError() << endl;
        return 1;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == nullptr) {
        cerr << "SDL_CreateRenderer error: " << SDL_GetError() << endl;
        return 1;
    }

    ///////////////////
    window_ui = SDL_CreateWindow("UIWINDOW", SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED, windowWidth, windowHeight,
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    context_ui = SDL_GL_CreateContext(window_ui);
    SDL_GL_MakeCurrent(window, context);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForOpenGL(window_ui, context_ui);
    ImGui_ImplOpenGL2_Init();
    ////////////////////

    initOpenGLBuffers();

    return 0;
}

void MyWindow::initOpenGLBuffers() {
    // create shaders
    const string vertexShaderSrc =
        "#version 100\n"
        "attribute vec3 position;\n"
        "attribute vec2 texCoord;\n"
        "varying vec2 fragTexCoord;\n"
        "void main() {\n"
        "  fragTexCoord = texCoord;\n"
        "  gl_Position = vec4(position,1);\n"
        "}\n";
    const string fragmentShaderSrc =
        "#version 100\n"
        "precision mediump float;\n"
        "uniform sampler2D tex;\n"
        "uniform float alpha;\n"
        "varying vec2 fragTexCoord;\n"
        "void main() {\n"
        "  vec4 sample = texture2D(tex, fragTexCoord);\n"
        "  if (alpha < 1.0) sample.w = sample.w * alpha;\n"
        "  gl_FragColor = sample;\n"
        "}\n";
    glData.screenShader =
        loadShaders(vertexShaderSrc.c_str(), fragmentShaderSrc.c_str());

    // init buffers for meshes
    glGenVertexArraysOES(1, &glData.VAO);  // extension for OpenGL ES 2.0
    glGenBuffers(1, &glData.VBO_V);
    glGenBuffers(1, &glData.VBO_T);
    glGenBuffers(1, &glData.VBO_F);
    glData.V.resize(4, 3);
    glData.V << -1, 1, 0, 1, 1, 0, 1, -1, 0, -1, -1, 0;
    glData.F.resize(2, 3);
    glData.F << 0, 1, 2, 2, 3, 0;
    glData.T.resize(4, 2);
    glData.T << 0, 0, 1, 0, 1, 1, 0, 1;
    // fill buffers
    glBindVertexArrayOES(glData.VAO);  // extension for OpenGL ES 2.0
    // position
    GLint id;
    glBindBuffer(GL_ARRAY_BUFFER, glData.VBO_V);
    glBufferData(GL_ARRAY_BUFFER, glData.V.size() * sizeof(float),
        glData.V.data(), GL_STATIC_DRAW);
    id = glGetAttribLocation(glData.screenShader, "position");
    glVertexAttribPointer(id, glData.V.cols(), GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(id);
    // texture coords
    glBindBuffer(GL_ARRAY_BUFFER, glData.VBO_T);
    glBufferData(GL_ARRAY_BUFFER, glData.T.size() * sizeof(float),
        glData.T.data(), GL_STATIC_DRAW);
    id = glGetAttribLocation(glData.screenShader, "texCoord");
    glVertexAttribPointer(id, glData.T.cols(), GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(id);
    // faces
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, glData.VBO_F);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, glData.F.size() * sizeof(unsigned int),
        glData.F.data(), GL_STATIC_DRAW);

    // init texture
    glGenTextures(1, &glData.screenTexture);
    glBindTexture(GL_TEXTURE_2D, glData.screenTexture);
    Imguc tmp(windowWidth, windowHeight, 4, 3);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, windowWidth, windowHeight, 0, GL_RGBA,
        GL_UNSIGNED_BYTE, tmp.data);
    // set the texture wrapping/filtering options (on the currently bound texture
    // object)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void MyWindow::destroyOpenGLBuffers() {
    // destroy buffers
    glDeleteVertexArraysOES(1, &glData.VAO);
    glDeleteBuffers(1, &glData.VBO_V);
    glDeleteBuffers(1, &glData.VBO_T);
    glDeleteBuffers(1, &glData.VBO_F);
}

#ifdef __EMSCRIPTEN__
void MyWindow::mainTickEmscripten(void* data) {
    static_cast<MyWindow*>(data)->mainTick();
}
#endif

void MyWindow::runLoop() {
#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg(mainTickEmscripten, (void*)this, -1, 1);
#else
    while (true) {
        if (mainTick()) break;
    }
#endif
}

void MyWindow::destroy()
{
    destroyOpenGLBuffers();
    SDL_DestroyRenderer(renderer);
    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
}


bool MyWindow::mainTick() {
    SDL_Event event;
    bool show_another_window = true;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    int lastTimestamp = event.motion.timestamp;
    bool quit = false;
    bool done = false;
    

    while (SDL_PollEvent(&event))
    {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window))
        {
            quit = true;
            break;
        }
        switch (event.type) {
        case SDL_QUIT:
            quit = true;
            break;
        case SDL_KEYUP:
        case SDL_KEYDOWN: {
            MyKeyEvent keyEvent;
            if (event.key.keysym.mod & KMOD_SHIFT) keyEvent.shiftModifier = true;
            if (event.key.keysym.mod & KMOD_CTRL) keyEvent.ctrlModifier = true;
            if (event.key.keysym.mod & KMOD_ALT) keyEvent.altModifier = true;
            if (event.key.keysym.mod & KMOD_NONE) keyEvent.noModifier = true;
            keyEvent.key = event.key.keysym.sym;
            lastKeyEvent = keyEvent;  // store this for usage in mouse events
            if (event.type == SDL_KEYDOWN)
                keyPressEvent(keyEvent);
            else
                keyReleaseEvent(keyEvent);

            this->keyEvent(keyEvent);
        } break;
        case SDL_MOUSEMOTION:
        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP: {
            if (!simulateMouseEventByTouch &&
                event.button.which == SDL_TOUCH_MOUSEID)
                break;
            MyMouseEvent mouseEvent;
            mouseEvent.pos = Vector2d(event.button.x, event.button.y);
            mouseEvent.shiftModifier = lastKeyEvent.shiftModifier;
            mouseEvent.ctrlModifier = lastKeyEvent.ctrlModifier;
            mouseEvent.altModifier = lastKeyEvent.altModifier;
            mouseEvent.noModifier = lastKeyEvent.noModifier;

            if (event.type == SDL_MOUSEMOTION) {
                mouseEvent.leftButton = event.motion.state & SDL_BUTTON_LMASK;
                mouseEvent.middleButton = event.motion.state & SDL_BUTTON_MMASK;
                mouseEvent.rightButton = event.motion.state & SDL_BUTTON_RMASK;
                //            mouseEvent.rightButton = false; // temporary
            }
            else {
                mouseEvent.leftButton = event.button.button == SDL_BUTTON_LEFT;
                mouseEvent.middleButton = event.button.button == SDL_BUTTON_MIDDLE;
                mouseEvent.rightButton = event.button.button == SDL_BUTTON_RIGHT;
                //            mouseEvent.rightButton = false; // temporary
                mouseEvent.numClicks = event.button.clicks;
            }

            if (event.type == SDL_MOUSEBUTTONDOWN)
                lastPressTimestamp = event.button.timestamp;
            else if (event.type == SDL_MOUSEBUTTONUP)
                mouseEvent.pressReleaseDurationMs =
                event.button.timestamp - lastPressTimestamp;

            if (event.type == SDL_MOUSEMOTION)
                mouseMoveEvent(mouseEvent);
            else if (event.type == SDL_MOUSEBUTTONDOWN)
                mousePressEvent(mouseEvent);
            else
                mouseReleaseEvent(mouseEvent);

            this->mouseEvent(mouseEvent);
        } break;
        case SDL_FINGERMOTION:
        case SDL_FINGERDOWN:
        case SDL_FINGERUP: {
            MyFingerEvent fingerEvent;
            // event.tfinger.x and event.tfinger.y are normalized to [0, 1]
            fingerEvent.pos = Vector2d(event.tfinger.x * windowWidth,
                event.tfinger.y * windowHeight);
            fingerEvent.fingerId = event.tfinger.fingerId;

            if (event.tfinger.type == SDL_FINGERDOWN)
                currFingerIds.insert(fingerEvent.fingerId);
            else if (event.tfinger.type == SDL_FINGERUP)
                currFingerIds.erase(fingerEvent.fingerId);

            fingerEvent.numFingers = currFingerIds.size();

            if (event.tfinger.type == SDL_FINGERDOWN)
                fingerPressEvent(fingerEvent);
            else if (event.tfinger.type == SDL_FINGERUP)
                fingerReleaseEvent(fingerEvent);
            else
                fingerMoveEvent(fingerEvent);

            this->fingerEvent(fingerEvent);
        } break;
        }
    }
    if (show_another_window)
    {
    SDL_GL_MakeCurrent(window_ui, context_ui);
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Setting UI", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)

    ImGui::Checkbox("", &key1);
    ImGui::SameLine();
    if (ImGui::Button("DRAW_OUTLINE"))
    {
        changekey1value();
    }

    ImGui::Checkbox("", &key2);
    ImGui::SameLine();
    if (ImGui::Button("DEFORM_MODE"))
    {
        changekey2value();
    }

    ImGui::Checkbox("", &key3);
    ImGui::SameLine();
    if (ImGui::Button("ANIMATE_MODE"))
    {
        changekey3value();
    }

    ImGui::Checkbox("", &key4);
    ImGui::SameLine();
    if (ImGui::Button("REGION_SWAP_MODE"))
    {
        changekey4value();
    }

    ImGui::Text("Use Custom Functoin?");
    ImGui::Checkbox("", &smline);
    ImGui::SameLine();
    if (ImGui::Button("Smoothing Line(Beta)"))
    {
        changemkvalue();
    }

    ImGui::Checkbox("", &mksk);
    ImGui::SameLine();
    if (ImGui::Button("Make Skeleton(Beta"))
    {
        changesmvalue();
    }

    if (ImGui::Button("End Setting"))
    {
        show_another_window = false;
        SDL_HideWindow(window_ui);
        /*SDL_DestroyRenderer(renderer_ui);
        SDL_GL_DeleteContext(context_ui);
        SDL_DestroyWindow(window_ui);*/
    }


    ImGui::End();

    ImGui::Render();
    glViewport(0, 0, (int)ImGui::GetIO().DisplaySize.x, (int)ImGui::GetIO().DisplaySize.y);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    //glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context where shaders may be bound
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window_ui);
    }

    SDL_GL_MakeCurrent(window, context);
    if (paintEvent()) 
    {
        SDL_GL_SwapWindow(window);
    }
    

    return quit;
}

SDL_Renderer* MyWindow::getRenderer() const { return renderer; }

SDL_Window* MyWindow::getWindow() const { return window; }

SDL_GLContext MyWindow::getContext() const
{
    return context;
}

const MyWindowGLData& MyWindow::getGLData() const { return glData; }

void MyWindow::setMouseEventsSimulationByTouch(bool enable) {
    simulateMouseEventByTouch = enable;
}

int MyWindow::getWidth() const { return windowWidth; }

int MyWindow::getHeight() const { return windowHeight; }

void MyWindow::setKeyboardEventState(bool enabled) {
    auto state = enabled ? SDL_ENABLE : SDL_DISABLE;
    SDL_EventState(SDL_TEXTINPUT, state);
    SDL_EventState(SDL_KEYDOWN, state);
    SDL_EventState(SDL_KEYUP, state);
}

void MyWindow::enableKeyboardEvents() { setKeyboardEventState(true); }

void MyWindow::disableKeyboardEvents() { setKeyboardEventState(false); }

bool MyWindow::key1value()
{
    return key1;
}

void MyWindow::changekey1value()
{
    if (key1value() == true)
        key1 = false;
    else
        key1 = true;
}

bool MyWindow::key2value()
{
    return key2;
}

void MyWindow::changekey2value()
{
    if (key2value() == true)
        key2 = false;
    else
        key2 = true;
}

bool MyWindow::key3value()
{
    return key3;
}

void MyWindow::changekey3value()
{
    if (key3value() == true)
        key3 = false;
    else
        key3 = true;
}

bool MyWindow::key4value()
{
    return key3;
}

void MyWindow::changekey4value()
{
    if (key4value() == true)
        key4 = false;
    else
        key4 = true;
}

bool MyWindow::ShowUI()
{
    return show_another_window;
}

void MyWindow::ChangeUI()
{
    if (ShowUI() == true)
        show_another_window = false;
    else
    {
        show_another_window = true;
        SDL_ShowWindow(window_ui);
    }
}

bool MyWindow::mkvalue()
{
    return mksk;
}

void MyWindow::changemkvalue()
{
    if (mkvalue() == true)
        mksk = false;
    else
    {
        mksk = true;
    }
}

bool MyWindow::smvalue()
{
    return smline;
}

void MyWindow::changesmvalue()
{
    if (smvalue() == true)
        smline = false;
    else
    {
        smline = true;
    }
}