#pragma once

#include "Renderer.h"
#include "Physics.h"

#include <vector>

// Game State
enum GameState
{
    Aiming,    // Bird on slingshot, can be dragged
    Flying,    // Bird launched, physics running
    Settled    // Bird stopped, waiting for reset
};

// Bird 
struct Bird
{
    RigidBody  body;
    Vec2       anchorPos;      // Slingshot fork
    Vec2       initialPos;     // Initial position before launch
    Color      color;
    bool       launched = false;
    float      pullDistance = 0.f;
    
    void init(float x, float y, float radius, const Color& c);
};

// Brick 
struct Brick
{
    RigidBody  body;
    Color      color;
    int        id;
    
    void init(float x, float y, float w, float h, const Color& c, int brickId);
};

// Pig
struct Pig
{
    RigidBody  body;
    int        id = 0;

    void init(float x, float y, float radius, int pigId);
};

// Game 
class Game
{
public:
    Game();
    
    void init(float screenW, float screenH, float groundY);
    void setViewportSize(float screenW, float screenH);
    void reset();
    
    void physicsStep(float dt);
    void render(Renderer2D& renderer, float screenW, float screenH);
    
    // Input handling
    void onMousePress(float worldX, float worldY, float birdRadius);
    void onMouseDrag(float worldX, float worldY);
    void onMouseRelease(float speedScale);
    
    void togglePause();
    bool isPaused() const { return paused_; }
    
    GameState getState() const { return state_; }
    
    PhysicsWorld& getPhysicsWorld() { return physics_; }
    
private:
    PhysicsWorld physics_;
    
    Bird   bird_;
    std::vector<Brick> bricks_;
    std::vector<Pig>   pigs_;
    RigidBody          groundBody_;
    
    Vec2   groundSize_;
    float  groundY_;
    float  screenW_;
    float  screenH_;
    
    GameState state_ = Aiming;
    bool      paused_ = false;
    
    // Dragging state
    bool      dragging_ = false;
    Vec2      pullPoint_;
    
    // Launch effect state
    bool      pendingLaunchFx_ = false;
    Vec2      pendingLaunchFxPos_;
    std::vector<Vec2> pendingPigFxPositions_;
    
    // Coordinate conversion helpers
    static Vec2 bodyToDrawPos(const RigidBody& body);
    static Vec2 bodyToDrawSize(const RigidBody& body);
    
    // Scene setup
    void setupScene();

    void rebuildPhysicsBodies();
    void syncBodiesFromPhysics();

    /// Pig hit
    void removePigsHitThisFrame();
    bool allDynamicBodiesSettled() const;

    int settleFrames_ = 0;
};
