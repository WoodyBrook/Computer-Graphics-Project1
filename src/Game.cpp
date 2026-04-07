#include "Game.h"

#include <cmath>
#include <algorithm>
#include <vector>

namespace
{

bool circleCircleOverlap(const RigidBody& a, const RigidBody& b)
{
    if (a.shape != RigidBody::Circle || b.shape != RigidBody::Circle)
        return false;
    Vec2 d = b.position - a.position;
    float r = a.radius + b.radius;
    return (d.x * d.x + d.y * d.y) <= r * r;
}

/// Circle vs rotated AABB (same test as PhysicsWorld::detectCircleRect).
float speedSquared(Vec2 v)
{
    return v.x * v.x + v.y * v.y;
}

} // namespace

// ──── Bird initialization ────
void Bird::init(float x, float y, float radius, const Color& c)
{
    body.shape = RigidBody::Circle;
    body.position = Vec2(x, y);
    body.velocity = Vec2(0.f, 0.f);
    body.rotation = 0.f;
    body.angularVel = 0.f;
    body.radius = radius;
    body.halfSize = Vec2(radius, radius);
    body.isStatic = true;  // Static while aiming
    body.computeMassProperties(1.f);
    body.restitution = 0.25f;  // Lower restitution for less bouncing
    body.friction = 0.3f;      // Higher friction for stability
    
    initialPos = Vec2(x, y);
    // Anchor is the slingshot stem top. Keep it clearly below the bird center so the
    // pouch sits around the lower half of the bird and feels nested in the bands.
    anchorPos = Vec2(x, y + 16.f);
    color = c;
    launched = false;
    pullDistance = 0.f;
}

// ──── Brick initialization ────
void Brick::init(float x, float y, float w, float h, const Color& c, int brickId)
{
    body.shape = RigidBody::Rect;
    body.position = Vec2(x, y);
    body.halfSize = Vec2(w * 0.5f, h * 0.5f);
    body.isStatic = false;
    body.computeMassProperties(0.8f);
    body.restitution = 0.25f;  // Lower restitution for less bouncing
    body.friction = 0.5f;      // Higher friction for stability when stacking
    
    color = c;
    id = brickId;
}

// ──── Pig initialization ────
void Pig::init(float x, float y, float radius, int pigId)
{
    body.shape = RigidBody::Circle;
    body.position = Vec2(x, y);
    body.velocity = Vec2(0.f, 0.f);
    body.rotation = 0.f;
    body.angularVel = 0.f;
    body.radius = radius;
    body.halfSize = Vec2(radius, radius);
    body.isStatic = false;
    body.computeMassProperties(0.9f);
    body.restitution = 0.2f;
    body.friction = 0.45f;

    id = pigId;
}

// ──── Game ────
Game::Game()
{
}

void Game::init(float screenW, float screenH, float groundY)
{
    setViewportSize(screenW, screenH);
    groundY_ = groundY;

    setupScene();
}

void Game::setViewportSize(float screenW, float screenH)
{
    screenW_ = screenW;
    screenH_ = screenH;
}

void Game::setupScene()
{
    pendingLaunchFx_ = false;
    pendingLaunchFxPos_ = Vec2(0.f, 0.f);
    pendingPigFxPositions_.clear();
    settleFrames_ = 0;

    // Keep the rendered ground and the physical floor aligned at the same top edge.
    groundSize_ = Vec2(std::max(screenW_ * 4.f, 6000.f), 548.f);
    groundBody_.shape = RigidBody::Rect;
    groundBody_.position = Vec2(screenW_ * 0.5f, groundY_ + groundSize_.y * 0.5f);
    groundBody_.velocity = Vec2(0.f, 0.f);
    groundBody_.rotation = 0.f;
    groundBody_.angularVel = 0.f;
    groundBody_.halfSize = groundSize_ * 0.5f;
    groundBody_.radius = 0.f;
    groundBody_.isStatic = true;
    groundBody_.isGround = true;
    groundBody_.restitution = 0.2f;
    groundBody_.friction = 0.6f;
    groundBody_.computeMassProperties(1.f);
    
    // Initialize bird so the full pull radius fits above the ground, which makes aiming
    // feel natural in all directions instead of getting distorted near the floor.
    float birdRadius = 20.f;
    float birdX = 200.f;
    const float kBirdNestAboveGround = 110.f;
    float birdY = groundY_ - birdRadius - kBirdNestAboveGround;
    bird_.init(birdX, birdY, birdRadius, Colors::Red);
    // Match the current slingshot drawing geometry: stemHeight is 82 in Renderer.cpp.
    // Keeping the stem top fixed relative to the ground makes the bird/sling relationship stable.
    bird_.anchorPos = Vec2(birdX, groundY_ - 82.f);
    
    // Initialize brick stack (3x3 pyramid)
    float brickW = 60.f;
    float brickH = 30.f;
    float startX = 800.f;
    // Spawn the tower already resting on the ground so launching the bird does not
    // introduce an artificial "settling" drop that nudges the pig off the stack.
    float startY = groundY_ - brickH * 0.5f;
    
    bricks_.clear();
    int brickId = 0;
    
    // Bottom layer: 3 bricks
    for (int col = 0; col < 3; ++col)
    {
        Brick brick;
        float x = startX + col * brickW;
        float y = startY;
        brick.init(x, y, brickW, brickH, Colors::pastelFromId(brickId), brickId);
        bricks_.push_back(brick);
        ++brickId;
    }
    
    // Middle layer: 2 bricks
    startY -= brickH;
    startX += brickW * 0.5f;
    for (int col = 0; col < 2; ++col)
    {
        Brick brick;
        float x = startX + col * brickW;
        float y = startY;
        brick.init(x, y, brickW, brickH, Colors::pastelFromId(brickId), brickId);
        bricks_.push_back(brick);
        ++brickId;
    }
    
    // Top layer: 1 brick
    startY -= brickH;
    startX += brickW * 0.5f;
    Brick topBrick;
    topBrick.init(startX, startY, brickW, brickH, Colors::pastelFromId(brickId), brickId);
    bricks_.push_back(topBrick);

    // Green pig on top of the pyramid
    pigs_.clear();
    float pigRadius = 18.f;
    float pigX = startX;
    float pigY = startY - brickH * 0.5f - pigRadius;
    Pig pig;
    pig.init(pigX, pigY, pigRadius, 0);
    pigs_.push_back(pig);

    rebuildPhysicsBodies();
    
    state_ = Aiming;
    paused_ = false;
    dragging_ = false;
}

void Game::reset()
{
    setupScene();
}

void Game::rebuildPhysicsBodies()
{
    physics_.clear();
    physics_.addBody(bird_.body);
    for (Brick& brick : bricks_)
        physics_.addBody(brick.body);
    for (Pig& p : pigs_)
        physics_.addBody(p.body);
    physics_.addBody(groundBody_);
}

void Game::syncBodiesFromPhysics()
{
    auto& bodies = physics_.bodies();
    if (bodies.empty())
        return;

    bird_.body = bodies[0];

    const size_t nBricks = bricks_.size();
    for (size_t i = 0; i < nBricks; ++i)
    {
        if (bodies.size() > 1 + i)
            bricks_[i].body = bodies[1 + i];
    }

    const size_t nPigs = pigs_.size();
    for (size_t j = 0; j < nPigs; ++j)
    {
        if (bodies.size() > 1 + nBricks + j)
            pigs_[j].body = bodies[1 + nBricks + j];
    }
}

void Game::removePigsHitThisFrame()
{
    if (pigs_.empty())
        return;

    const float kPigHitSpeed = 45.f;
    const float kPigHitSpeedSq = kPigHitSpeed * kPigHitSpeed;
    const float kGroundTouchSlop = 2.f;

    std::vector<char> dead(pigs_.size(), 0);
    for (size_t j = 0; j < pigs_.size(); ++j)
    {
        const RigidBody& pigBody = pigs_[j].body;

        if (pigBody.position.y + pigBody.radius >= groundY_ - kGroundTouchSlop)
        {
            dead[j] = 1;
            continue;
        }

        if (circleCircleOverlap(bird_.body, pigBody) &&
            speedSquared(bird_.body.velocity - pigBody.velocity) >= kPigHitSpeedSq)
        {
            dead[j] = 1;
            continue;
        }
    }

    bool anyDead = false;
    for (char d : dead)
    {
        if (d)
        {
            anyDead = true;
            break;
        }
    }
    if (!anyDead)
        return;

    std::vector<Pig> kept;
    kept.reserve(pigs_.size());
    for (size_t j = 0; j < pigs_.size(); ++j)
    {
        if (!dead[j])
            kept.push_back(pigs_[j]);
        else
            pendingPigFxPositions_.push_back(pigs_[j].body.position);
    }
    pigs_.swap(kept);
    rebuildPhysicsBodies();
}

bool Game::allDynamicBodiesSettled() const
{
    const auto& bodies = physics_.bodies();
    bool hasDynamic = false;

    for (const RigidBody& body : bodies)
    {
        if (body.isStatic)
            continue;

        hasDynamic = true;
        if (body.velocity.length() > 4.f || std::abs(body.angularVel) > 0.05f)
            return false;
    }

    return hasDynamic;
}

// ──── Coordinate conversion ────
Vec2 Game::bodyToDrawPos(const RigidBody& body)
{
    // Convert center + halfSize to draw position (top-left corner)
    if (body.shape == RigidBody::Circle)
    {
        return Vec2(body.position.x - body.radius, body.position.y - body.radius);
    }
    else
    {
        return Vec2(body.position.x - body.halfSize.x, body.position.y - body.halfSize.y);
    }
}

Vec2 Game::bodyToDrawSize(const RigidBody& body)
{
    if (body.shape == RigidBody::Circle)
    {
        return Vec2(body.radius * 2.f, body.radius * 2.f);
    }
    else
    {
        return Vec2(body.halfSize.x * 2.f, body.halfSize.y * 2.f);
    }
}

// ──── Physics step ────
void Game::physicsStep(float dt)
{
    if (paused_)
        return;
    
    if (state_ == Flying)
    {
        physics_.step(dt);
        syncBodiesFromPhysics();
        removePigsHitThisFrame();

        // Only stop the simulation when the entire scene has been calm for a short
        // moment, otherwise the old bird-only rule freezes half-collapsed towers.
        if (allDynamicBodiesSettled())
            ++settleFrames_;
        else
            settleFrames_ = 0;

        if (settleFrames_ >= 18)
            state_ = Settled;
    }
}

// ──── Render ────
void Game::render(Renderer2D& renderer, float screenW, float screenH)
{
    // Draw background (grass strip)
    renderer.drawBackground(groundY_, screenW);
    
    // Draw ground
    renderer.drawGround(groundY_, screenW);
    
    // Draw bricks
    for (const Brick& brick : bricks_)
    {
        Vec2 drawPos = bodyToDrawPos(brick.body);
        Vec2 drawSize = bodyToDrawSize(brick.body);
        renderer.drawBrick(drawPos, drawSize, brick.color, brick.body.rotation);
    }

    for (const Pig& pig : pigs_)
    {
        if (pig.body.shape == RigidBody::Circle)
            renderer.drawPig(pig.body.position, pig.body.radius, pig.body.rotation);
    }
    
    // Draw bird
    if (bird_.body.shape == RigidBody::Circle)
    {
        renderer.drawBird(bird_.body.position, bird_.body.radius, bird_.color, bird_.body.rotation);
    }
    
    // Draw slingshot when aiming
    if (state_ == Aiming)
    {
        Vec2 pullPt = dragging_ ? pullPoint_ : bird_.body.position;
        renderer.drawSlingshot(bird_.anchorPos, pullPt);
        
        // Draw trajectory preview when dragging
        if (dragging_)
        {
            Vec2 launchVel = (bird_.initialPos - pullPoint_) * 8.f;
            renderer.drawTrajectory(bird_.body.position, launchVel, 500.f);
        }
    }
    
    // Consume launch effect
    if (pendingLaunchFx_)
    {
        renderer.spawnParticles(pendingLaunchFxPos_, 12, bird_.color);
        pendingLaunchFx_ = false;
    }

    for (const Vec2& pigFxPos : pendingPigFxPositions_)
    {
        renderer.spawnParticles(pigFxPos, 16, Colors::Green);
    }
    pendingPigFxPositions_.clear();

    if (paused_)
    {
        renderer.drawPausedOverlay(screenW, screenH);
    }
}

// ──── Input handling ────
void Game::onMousePress(float worldX, float worldY, float birdRadius)
{
    if (paused_ || state_ != Aiming)
        return;
    
    // Check if mouse is near the bird
    Vec2 mousePos(worldX, worldY);
    Vec2 birdPos = bird_.body.position;
    float dist = (mousePos - birdPos).length();
    
    if (dist < birdRadius * 2.5f)
    {
        dragging_ = true;
        pullPoint_ = mousePos;
    }
}

void Game::onMouseDrag(float worldX, float worldY)
{
    if (paused_ || !dragging_ || state_ != Aiming)
        return;
    
    pullPoint_ = Vec2(worldX, worldY);
    
    const float r = bird_.body.radius;
    const float margin = 8.f;
    // Keep bird bottom above the ground plane (same line physics uses); still allow a
    // useful downward pull because the nest sits higher than the ground contact line.
    const float maxBirdCenterY = groundY_ - r - 2.f;
    const float pullYHi = std::min(screenH_ - r - margin, maxBirdCenterY);

    pullPoint_.x = std::clamp(pullPoint_.x, margin, screenW_ - margin);
    pullPoint_.y = std::clamp(pullPoint_.y, margin, pullYHi);
    
    // Keep the pull radius shorter than the bird's rest height above the ground so aiming
    // can sweep through the full circle without the ground clamp distorting the angle.
    float maxPull = 90.f;
    Vec2 diff = pullPoint_ - bird_.initialPos;
    float dist = diff.length();
    
    if (dist > maxPull)
    {
        pullPoint_ = bird_.initialPos + diff * (maxPull / dist);
        pullPoint_.x = std::clamp(pullPoint_.x, margin, screenW_ - margin);
        pullPoint_.y = std::clamp(pullPoint_.y, margin, pullYHi);
    }
    
    // Update bird position to follow pull point
    bird_.body.position = pullPoint_;
    
    // Update in physics world
    if (physics_.bodies().size() > 0)
    {
        physics_.bodies()[0].position = pullPoint_;
    }
}

void Game::onMouseRelease(float speedScale)
{
    if (paused_ || !dragging_ || state_ != Aiming)
        return;
    
    dragging_ = false;
    
    // Stretch from rest: zero velocity when pullPoint == initialPos
    Vec2 launchDir = bird_.initialPos - pullPoint_;
    float pullDist = launchDir.length();
    
    if (pullDist > 10.f)  // Minimum pull distance to launch
    {
        Vec2 launchVel = launchDir * speedScale;
        
        // Set bird velocity and recompute mass (was zero while static)
        bird_.body.isStatic = false;
        bird_.body.computeMassProperties(1.f);
        bird_.body.velocity = launchVel;
        bird_.launched = true;
        bird_.pullDistance = pullDist;
        
        // Update in physics world
        if (physics_.bodies().size() > 0)
        {
            physics_.bodies()[0].isStatic = false;
            physics_.bodies()[0].computeMassProperties(1.f);
            physics_.bodies()[0].velocity = launchVel;
        }
        
        state_ = Flying;
        
        // Set pending launch effect flag
        pendingLaunchFx_ = true;
        pendingLaunchFxPos_ = bird_.body.position;
    }
    else
    {
        // Not enough pull, reset bird position
        bird_.body.position = bird_.initialPos;
        if (physics_.bodies().size() > 0)
        {
            physics_.bodies()[0].position = bird_.initialPos;
        }
    }
}

void Game::togglePause()
{
    paused_ = !paused_;

    // Cancelling an active aim drag avoids "pause while aiming, release to launch"
    // and restores the bird to its neutral slingshot position.
    if (paused_ && state_ == Aiming && dragging_)
    {
        dragging_ = false;
        pullPoint_ = bird_.initialPos;
        bird_.body.position = bird_.initialPos;
        bird_.body.velocity = Vec2(0.f, 0.f);
        bird_.body.angularVel = 0.f;

        if (!physics_.bodies().empty())
        {
            physics_.bodies()[0].position = bird_.initialPos;
            physics_.bodies()[0].velocity = Vec2(0.f, 0.f);
            physics_.bodies()[0].angularVel = 0.f;
        }
    }
}