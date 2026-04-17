#include "Renderer.h"
#include "Game.h"

#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

#include <iostream>
#include <cstdlib>

// Constants
const int kWindowWidth  = 1280;
const int kWindowHeight = 720;
const float kGroundY    = 632.f;  // Top edge of the ground (floorY)
const float kSpeedScale = 8.f;    // Launch speed multiplier

// Global state
static Renderer2D gRenderer;
static Game gGame;
static float gScaleX = 1.f;
static float gScaleY = 1.f;

static bool updateWindowMetrics(GLFWwindow* window, int& winW, int& winH, int& fbW, int& fbH)
{
    glfwGetWindowSize(window, &winW, &winH);
    glfwGetFramebufferSize(window, &fbW, &fbH);

    if (winW <= 0 || winH <= 0 || fbW <= 0 || fbH <= 0)
        return false;

    gScaleX = static_cast<float>(fbW) / static_cast<float>(winW);
    gScaleY = static_cast<float>(fbH) / static_cast<float>(winH);
    gGame.setViewportSize(static_cast<float>(fbW), static_cast<float>(fbH));
    return true;
}

// GLFW callbacks
static void onErrorCallback(int error, const char* description)
{
    std::cerr << "GLFW Error (" << error << "): " << description << "\n";
}

static void onKeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
    else if (key == GLFW_KEY_R && action == GLFW_PRESS)
    {
        gGame.reset();
    }
    else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
    {
        gGame.togglePause();
    }
}

static void onMouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    if (button != GLFW_MOUSE_BUTTON_LEFT)
        return;
    
    double mx, my;
    glfwGetCursorPos(window, &mx, &my);
    
    // Apply Retina scaling
    float worldX = static_cast<float>(mx) * gScaleX;
    float worldY = static_cast<float>(my) * gScaleY;
    
    if (action == GLFW_PRESS)
    {
        // Get bird radius from game
        float birdRadius = 20.f;  // Default, could get from game
        gGame.onMousePress(worldX, worldY, birdRadius);
    }
    else if (action == GLFW_RELEASE)
    {
        gGame.onMouseRelease(kSpeedScale);
    }
}

static void onCursorPosCallback(GLFWwindow* window, double mx, double my)
{
    // Apply Retina scaling
    float worldX = static_cast<float>(mx) * gScaleX;
    float worldY = static_cast<float>(my) * gScaleY;
    
    gGame.onMouseDrag(worldX, worldY);
}

// Entry point 
int main()
{
    // Initialize GLFW
    glfwSetErrorCallback(onErrorCallback);
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return EXIT_FAILURE;
    }

    // Configure OpenGL 3.3 Core context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);  // Optional: MSAA

    // Create window
    GLFWwindow* window = glfwCreateWindow(kWindowWidth, kWindowHeight, 
                                          "Angry Birds - 2D Physics", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return EXIT_FAILURE;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync

    // Get window and framebuffer sizes
    int winW, winH, fbW, fbH;
    if (!updateWindowMetrics(window, winW, winH, fbW, fbH))
    {
        std::cerr << "Window size unavailable\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    // Register callbacks
    glfwSetKeyCallback(window, onKeyCallback);
    glfwSetMouseButtonCallback(window, onMouseButtonCallback);
    glfwSetCursorPosCallback(window, onCursorPosCallback);

    // Initialize renderer
    gRenderer.init();

    // Initialize game
    gGame.init(static_cast<float>(fbW), static_cast<float>(fbH), kGroundY);

    // Main loop with fixed physics timestep
    const double physDt = 1.0 / 60.0;  // Fixed physics step
    double accumulator = 0.0;
    double lastTime = glfwGetTime();
    double frameTime = 0.0;
    
    while (!glfwWindowShouldClose(window))
    {
        double now = glfwGetTime();
        frameTime = std::min(now - lastTime, 0.25);  // Clamp to prevent spiral of death
        lastTime = now;
        accumulator += frameTime;
        
        // Fixed timestep physics updates
        while (accumulator >= physDt)
        {
            gGame.physicsStep(static_cast<float>(physDt));
            accumulator -= physDt;
        }
        
        // Keep logical size, framebuffer size, input scaling, and game viewport in sync.
        if (!updateWindowMetrics(window, winW, winH, fbW, fbH))
        {
            glfwPollEvents();
            continue;
        }
        
        // Begin frame
        gRenderer.beginFrame(fbW, fbH);
        
        // Render game scene
        gGame.render(gRenderer, static_cast<float>(fbW), static_cast<float>(fbH));
        
        // Update and draw particles
        gRenderer.updateAndDrawParticles(static_cast<float>(frameTime));
        
        // End frame
        gRenderer.flush();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    gRenderer.shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();

    return EXIT_SUCCESS;
}
