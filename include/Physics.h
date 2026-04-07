#pragma once

#include "Renderer.h"  // Include for Vec2 and Color definitions
#include <vector>

// ──── Collision Info ────
struct CollisionInfo
{
    Vec2  normal;       // A → B direction
    float penetration;
    Vec2  contactPoint;
    bool  hasCollision = false;
};

// ──── Rigid Body (value semantics) ────
struct RigidBody
{
    Vec2  position;          // Center point
    Vec2  velocity;
    float rotation     = 0.f;  // Radians
    float angularVel   = 0.f;
    float mass         = 1.f;
    float invMass      = 1.f;
    float inertia      = 1.f;
    float invInertia   = 1.f;
    Vec2  halfSize;          // Rectangle half-width, half-height
    float radius       = 0.f; // Circle radius (0 if rectangle)
    
    enum Shape { Circle, Rect } shape = Rect;
    bool  isStatic     = false;
    bool  isGround     = false; // Static rect that represents the floor material
    float restitution  = 0.25f;  // Coefficient of restitution (bounce)
    float friction     = 0.5f;  // Coulomb friction coefficient
    
    // Compute mass properties from shape
    void computeMassProperties(float density);
};

// ──── Physics World ────
class PhysicsWorld
{
public:
    PhysicsWorld();
    
    void addBody(const RigidBody& body);
    void clear();
    void step(float dt);
    
    std::vector<RigidBody>& bodies();
    const std::vector<RigidBody>& bodies() const;
    
    // Collision iteration count (higher = more stable stacking)
    int collisionIterations = 6;
    
private:
    std::vector<RigidBody> bodies_;
    float gravity_ = 500.f;  // Pixels/s^2, y-down coordinate system
    
    // Integration: apply gravity and damping
    void integrate(RigidBody& body, float dt);
    
    // Collision detection
    bool detectCollision(const RigidBody& a, const RigidBody& b, CollisionInfo& info);
    bool detectCircleRect(const RigidBody& circle, const RigidBody& rect, CollisionInfo& info);
    bool detectRectRect(const RigidBody& a, const RigidBody& b, CollisionInfo& info);
    
    // Collision response
    void resolveCollision(RigidBody& a, RigidBody& b, const CollisionInfo& info);
    
    // Position correction (Baumgarte stabilization)
    void correctPositions(RigidBody& a, RigidBody& b, const CollisionInfo& info);
};

// ──── Utility functions ────
inline float cross2D(Vec2 a, Vec2 b);  // 2D cross product (scalar)
inline Vec2  cross2D(Vec2 v, float s);  // Cross vector with scalar
inline Vec2  cross2D(float s, Vec2 v);  // Cross scalar with vector
inline float dot(Vec2 a, Vec2 b);
inline float lengthSquared(Vec2 v);
inline Vec2  normalize(Vec2 v);