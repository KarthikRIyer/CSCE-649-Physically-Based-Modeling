#include <iostream>
#include <fstream>
#include <vector>

#ifndef _GLIBCXX_USE_NANOSLEEP
#define _GLIBCXX_USE_NANOSLEEP
#endif
#include <thread>
#include <iostream>
#include <fstream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/gtc/type_ptr.hpp>

#include "GLSL.h"
#include "Program.h"
#include "Camera.h"
#include "MatrixStack.h"
#include "Shape.h"
#include "Scene.h"
#include "Texture.h"
#include "SimParams.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

using namespace std;
using namespace Eigen;

bool keyToggles[256] = {false}; // only for English keyboards!

GLFWwindow *window; // Main application window
string RESOURCE_DIR = ""; // Where the resources are loaded from
string DATA_DIR = ""; // where the data are loaded from

shared_ptr<Camera> camera;
shared_ptr<Program> progMesh;
shared_ptr<Scene> scene;
int texUnit = 1;

SimParams simParams;
const char* sceneNames[] = {"Drop", "Multi-slope", "Point Gravity", "Slope Wedge", "Saw", "Chladni"};
int currSceneIndex = 0;
static const char* currentScene = sceneNames[currSceneIndex];
int prevSceneIndex = 0;

// https://stackoverflow.com/questions/41470942/stop-infinite-loop-in-different-thread
std::atomic<bool> stop_flag;
std::atomic<bool> run_sim;
std::atomic<bool> sim_paused;

static void error_callback(int error, const char *description)
{
	cerr << description << endl;
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    auto& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard || io.WantTextInput) return;
	if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		stop_flag = true;
		glfwSetWindowShouldClose(window, GL_TRUE);
	}
}

static void char_callback(GLFWwindow *window, unsigned int key)
{
    auto& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard || io.WantTextInput) return;
	keyToggles[key] = !keyToggles[key];
    std::ofstream dummyOStream(nullptr);
	switch(key) {
		case 'h':
		    scene->updateSimParams(simParams);
			scene->step(dummyOStream, false);
			break;
		case 'r':
			scene->reset();
			break;
	}
}

static void cursor_position_callback(GLFWwindow* window, double xmouse, double ymouse)
{
    auto& io = ImGui::GetIO();
    if (io.WantCaptureMouse) return;
	int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	if(state == GLFW_PRESS) {
		camera->mouseMoved(xmouse, ymouse);
	}
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    auto& io = ImGui::GetIO();
    if (io.WantCaptureMouse) return;
	// Get the current mouse position.
	double xmouse, ymouse;
	glfwGetCursorPos(window, &xmouse, &ymouse);
	// Get current window size.
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	if(action == GLFW_PRESS) {
		bool shift = mods & GLFW_MOD_SHIFT;
		bool ctrl  = mods & GLFW_MOD_CONTROL;
		bool alt   = mods & GLFW_MOD_ALT;
		camera->mouseClicked(xmouse, ymouse, shift, ctrl, alt);
	}
}

static void init()
{
	GLSL::checkVersion();

	// Set background color
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	// Enable z-buffer test
	glEnable(GL_DEPTH_TEST);
	// Enable alpha blending
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    progMesh = make_shared<Program>();
    progMesh->setShaderNames(RESOURCE_DIR + "phongmodel_vert.glsl", RESOURCE_DIR + "phongmodel_frag.glsl");
    progMesh->setVerbose(true); // Set this to true when debugging.
    progMesh->init();
    progMesh->addAttribute("aPos");
    progMesh->addAttribute("aNor");
    progMesh->addAttribute("aTex");
    progMesh->addUniform("P");
    progMesh->addUniform("MV");
    progMesh->addUniform("ka");
    progMesh->addUniform("ks");
    progMesh->addUniform("s");
    progMesh->addUniform("kdTex");
    progMesh->setVerbose(false);

    // Bind the texture to texUnit 1.
    progMesh->bind();
    glUniform1i(progMesh->getUniform("kdTex"), texUnit);
    progMesh->unbind();
	
	camera = make_shared<Camera>();

	scene = make_shared<Scene>();
	scene->load(RESOURCE_DIR, DATA_DIR, texUnit);
	scene->tare();
	scene->init();

	// If there were any OpenGL errors, this will print something.
	// You can intersperse this line in your code to find the exact location
	// of your OpenGL error.
	GLSL::checkError(GET_FILE_LINE);
}

void render()
{
	// Get current frame buffer size.
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);
	
	// Use the window size for camera.
	glfwGetWindowSize(window, &width, &height);
	camera->setAspect((float)width/(float)height);

	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(keyToggles[(unsigned)'c']) {
		glEnable(GL_CULL_FACE);
	} else {
		glDisable(GL_CULL_FACE);
	}
	if(keyToggles[(unsigned)'z']) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	} else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	
	auto P = make_shared<MatrixStack>();
	auto MV = make_shared<MatrixStack>();
	
	// Apply camera transforms
	P->pushMatrix();
	camera->applyProjectionMatrix(P);
	MV->pushMatrix();
	camera->applyViewMatrix(MV);



	// Draw scene

    progMesh->bind();
	glUniformMatrix4fv(progMesh->getUniform("P"), 1, GL_FALSE, glm::value_ptr(P->topMatrix()));
    glUniformMatrix4fv(progMesh->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	MV->pushMatrix();
    scene->draw(MV, progMesh);
	MV->popMatrix();
    progMesh->unbind();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ImGui::Begin("Simulation params");
//    ImGui::SliderFloat("Wind strength", &simParams.windStrength, 0.0f, 5.0f);
//    ImGui::SliderFloat("Wind oscillation speed", &simParams.windOscilationSpeed, 0.0f, 10.0f);
//    ImGui::SliderFloat("Air Friction Factor", &simParams.airFrictionFactor, 0.0f, 5.0f);
    ImGui::SliderFloat("Restitution Coefficient", &simParams.restitutionCoeff, 0.0f, 1.0f);
    ImGui::SliderFloat("Friction Coefficient", &simParams.frictionCoeff, 0.0f, 5.0f);
    ImGui::SliderFloat("Initial Particle Velocity", &simParams.initialParticleVelocity, 0.0f, 50.0f);
    ImGui::SliderFloat("Initial Particle Velocity Randomness", &simParams.initialParticleVelocityRandomness, 0.0f, 20.0f);
    ImGui::SliderFloat("Chladni A", &simParams.chladniA, 0.0f, 5.0f);
    ImGui::SliderFloat("Chladni B", &simParams.chladniB, 0.0f, 5.0f);
    ImGui::SliderFloat("Chladni M", &simParams.chladniM, 0.0f, 2.0f);
    ImGui::SliderFloat("Chladni N", &simParams.chladniN, 0.0f, 2.0f);
//    ImGui::SliderFloat("Timestep", &simParams.timestep, 1e-3, 5e-2);

    if (ImGui::BeginCombo("Choose scene", currentScene)) {
        for (int n = 0; n < IM_ARRAYSIZE(sceneNames); n++) {
            bool is_selected = (currentScene == sceneNames[n]);
            if (ImGui::Selectable(sceneNames[n], is_selected)) {
                currentScene = sceneNames[n];
                currSceneIndex = n;
            }
            if (is_selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    if (ImGui::Button("Load Scene")) {
//    if (ImGui::Button("Load Scene") && prevSceneIndex != currSceneIndex) {
        std::cout<<"Clicked\n";
        prevSceneIndex = currSceneIndex;
        run_sim = false;
        keyToggles[(unsigned)' '] = false;
        while (!sim_paused) {}
        scene->cleanup();
        scene = make_shared<Scene>();
        scene->setSceneNum(currSceneIndex);
        scene->updateSimParams(simParams);
        scene->load(RESOURCE_DIR, DATA_DIR, texUnit);
        scene->tare();
        scene->init();
        scene->updateSimParams(simParams);
        sim_paused = false;
        run_sim = true;
    }

    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    //////////////////////////////////////////////////////
	// Cleanup
	//////////////////////////////////////////////////////
	
	// Pop stacks
	MV->popMatrix();
	P->popMatrix();

	GLSL::checkError(GET_FILE_LINE);
}

void simLoop()
{
	double t = 0;
	int n = 0;
	std::ofstream outputFile("output.txt");
	outputFile << scene->getTimestep() <<"\n";
	while(!stop_flag) {
	    if (run_sim) {
            auto t0 = std::chrono::system_clock::now();
            if(keyToggles[(unsigned)' ']) {
                scene->updateSimParams(simParams);
                scene->step(outputFile, keyToggles[(unsigned)'f']);
            }
            auto t1 = std::chrono::system_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
            t += dt*1e-3;
            n++;
//            double sleeptime = std::max(5000 - dt, 0.0);
            double sleeptime = std::max(scene->getTimestep() * 1e6 - dt, 0.0);
            this_thread::sleep_for(chrono::microseconds((long)sleeptime));

            if(keyToggles[(unsigned)' '] && keyToggles[(unsigned )'t']) {
                std::cout<<"===================================\n";
                std::cout<<"timestep: "<<scene->getTimestep()*1e3<<" ms\n";
                std::cout<<"processing time: "<<dt*1e-3<<" ms\n";
                std::cout<<"sleep time: "<<sleeptime*1e-3<<" ms\n";
            }
            if (!run_sim) sim_paused = true;
	    }
	}
}

int main(int argc, char **argv)
{
	if(argc < 3) {
		cout << "Please specify the resource and data directory." << endl;
		return 0;
	}
	RESOURCE_DIR = argv[1] + string("/");
    DATA_DIR = argv[2] + string("/");

	// Set error callback.
	glfwSetErrorCallback(error_callback);
	// Initialize the library.
	if(!glfwInit()) {
		return -1;
	}
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	// Create a windowed mode window and its OpenGL context.
	window = glfwCreateWindow(640*2, 480*2, "A2 - Particle Systems", NULL, NULL);
	if(!window) {
		glfwTerminate();
		return -1;
	}
	
	// Make the window's context current.
	glfwMakeContextCurrent(window);
	// Initialize GLEW.
	glewExperimental = true;
	if(glewInit() != GLEW_OK) {
		cerr << "Failed to initialize GLEW" << endl;
		return -1;
	}
	glGetError(); // A bug in glewInit() causes an error that we can safely ignore.
	cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
	cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
	// Set vsync.
	glfwSwapInterval(1);
	// Set keyboard callback.
	glfwSetKeyCallback(window, key_callback);
	// Set char callback.
	glfwSetCharCallback(window, char_callback);
	// Set cursor position callback.
	glfwSetCursorPosCallback(window, cursor_position_callback);
	// Set mouse button callback.
	glfwSetMouseButtonCallback(window, mouse_button_callback);

	// Initialize scene.
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 150");
    init();

	// Start simulation thread.
	stop_flag = false;
	run_sim = true;
	sim_paused = false;
	thread simThread(simLoop);
	// Loop until the user closes the window.
	while(!glfwWindowShouldClose(window)) {
		if(!glfwGetWindowAttrib(window, GLFW_ICONIFIED)) {
			// Render scene.
			render();
			// Swap front and back buffers.
			glfwSwapBuffers(window);
		}
		// Poll for and process events.
		glfwPollEvents();
	}
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
	// Quit program.
	stop_flag = true;
	simThread.join();
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}
