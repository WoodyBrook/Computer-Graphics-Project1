#include "Physics.h"
#include "Renderer.h"

#include <algorithm>
#include <cmath>

// ──── Utility functions ────

inline float cross2D(Vec2 a, Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}

inline Vec2 cross2D(Vec2 v, float s)
{
    return Vec2(s * v.y, -s * v.x);
}

inline Vec2 cross2D(float s, Vec2 v)
{
    return Vec2(-s * v.y, s * v.x);
}

inline float dot(Vec2 a, Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

inline float lengthSquared(Vec2 v)
{
    return v.x * v.x + v.y * v.y;
}

inline Vec2 normalize(Vec2 v)
{
    float len = v.length();
    return len > 1e-6f ? v / len : Vec2(0.f, 0.f);
}

// ──── RigidBody mass computation ────

void RigidBody::computeMassProperties(float density)
{
    if (isStatic)
    {
        mass = 0.f;
        invMass = 0.f;
        inertia = 0.f;
        invInertia = 0.f;
        return;
    }
    
    if (shape == RigidBody::Circle)
    {
        float r = radius;
        mass = density * 3.14159265359f * r * r;
        inertia = mass * r * r * 0.5f;
    }
    else // RigidBody::Rect
    {
        float w = halfSize.x * 2.f;
        float h = halfSize.y * 2.f;
        mass = density * w * h;
        inertia = mass * (w * w + h * h) / 12.f;
    }
    
    invMass = mass > 1e-6f ? 1.f / mass : 0.f;
    invInertia = inertia > 1e-6f ? 1.f / inertia : 0.f;
}

// ──── PhysicsWorld ────

PhysicsWorld::PhysicsWorld()
{
}

void PhysicsWorld::addBody(const RigidBody& body)
{
    bodies_.push_back(body);
}

void PhysicsWorld::clear()
{
    bodies_.clear();
}

std::vector<RigidBody>& PhysicsWorld::bodies()
{
    return bodies_;
}

const std::vector<RigidBody>& PhysicsWorld::bodies() const
{
    return bodies_;
}

void PhysicsWorld::integrate(RigidBody& body, float dt)
{
    if (body.isStatic)
        return;
    
    // Apply gravity (y-down coordinate system)
    body.velocity.y += gravity_ * dt;
    
    // Apply linear damping (slightly stronger for stability)
    body.velocity *= 0.998f;
    
    // Apply angular damping (stronger for stability)
    body.angularVel *= 0.992f;
    
    // Semi-implicit Euler integration
    body.position += body.velocity * dt;
    body.rotation += body.angularVel * dt;
    
    // Clamp maximum velocity to prevent tunneling
    float maxVel = 2000.f;
    float velLen = body.velocity.length();
    if (velLen > maxVel)
    {
        body.velocity = body.velocity * (maxVel / velLen);
    }
    
    // Clamp angular velocity
    float maxAngVel = 20.f;
    if (std::abs(body.angularVel) > maxAngVel)
    {
        body.angularVel = maxAngVel * (body.angularVel > 0.f ? 1.f : -1.f);
    }
    
}

// ──── Collision Detection ────

bool PhysicsWorld::detectCollision(const RigidBody& a, const RigidBody& b, CollisionInfo& info)
{
    if (a.isStatic && b.isStatic)
        return false;
    
    if (a.shape == RigidBody::Circle && b.shape == RigidBody::Rect)
        return detectCircleRect(a, b, info);
    else if (a.shape == RigidBody::Rect && b.shape == RigidBody::Circle)
    {
        bool coll = detectCircleRect(b, a, info);
        if (coll)
            info.normal = info.normal * -1.f;  // Flip normal direction
        return coll;
    }
    else if (a.shape == RigidBody::Rect && b.shape == RigidBody::Rect)
        return detectRectRect(a, b, info);
    else // Circle vs Circle
    {
        Vec2 diff = b.position - a.position;
        float dist = diff.length();
        float rSum = a.radius + b.radius;
        if (dist < rSum)
        {
            info.hasCollision = true;
            info.normal = dist > 1e-6f ? diff / dist : Vec2(0.f, 1.f);
            info.penetration = rSum - dist;
            info.contactPoint = a.position + info.normal * a.radius;
            return true;
        }
        return false;
    }
}

bool PhysicsWorld::detectCircleRect(const RigidBody& circle, const RigidBody& rect, CollisionInfo& info)
{
    // Transform circle center to rectangle's local coordinate system
    float c = std::cos(-rect.rotation);
    float s = std::sin(-rect.rotation);
    Vec2 localCircle;
    localCircle.x = (circle.position.x - rect.position.x) * c - (circle.position.y - rect.position.y) * s;
    localCircle.y = (circle.position.x - rect.position.x) * s + (circle.position.y - rect.position.y) * c;
    
    // Clamp to rectangle bounds
    Vec2 closest;
    closest.x = std::max(-rect.halfSize.x, std::min(localCircle.x, rect.halfSize.x));
    closest.y = std::max(-rect.halfSize.y, std::min(localCircle.y, rect.halfSize.y));
    
    // Check if circle center is inside rectangle
    bool inside = (localCircle.x == closest.x && localCircle.y == closest.y);
    if (inside)
    {
        // Push to nearest edge
        float dx = rect.halfSize.x - std::abs(localCircle.x);
        float dy = rect.halfSize.y - std::abs(localCircle.y);
        if (dx < dy)
        {
            closest.x = localCircle.x > 0.f ? rect.halfSize.x : -rect.halfSize.x;
        }
        else
        {
            closest.y = localCircle.y > 0.f ? rect.halfSize.y : -rect.halfSize.y;
        }
    }
    
    Vec2 localDiff = localCircle - closest;
    float distSq = localDiff.x * localDiff.x + localDiff.y * localDiff.y;
    float radius = circle.radius;
    
    if (distSq > radius * radius && !inside)
        return false;
    
    float dist = std::sqrt(distSq);
    
    // Transform back to world coordinates
    c = std::cos(rect.rotation);
    s = std::sin(rect.rotation);
    Vec2 worldClosest;
    worldClosest.x = rect.position.x + closest.x * c - closest.y * s;
    worldClosest.y = rect.position.y + closest.x * s + closest.y * c;
    
    Vec2 worldNormal;
    if (inside)
    {
        Vec2 localNormal = localDiff / (dist > 1e-6f ? dist : 1.f);
        worldNormal.x = localNormal.x * c - localNormal.y * s;
        worldNormal.y = localNormal.x * s + localNormal.y * c;
        info.penetration = radius + dist;
    }
    else
    {
        Vec2 localNormal = dist > 1e-6f ? localDiff / dist : Vec2(0.f, 1.f);
        worldNormal.x = localNormal.x * c - localNormal.y * s;
        worldNormal.y = localNormal.x * s + localNormal.y * c;
        info.penetration = radius - dist;
    }
    
    info.hasCollision = true;
    info.normal = worldNormal * -1.f;  // Flip: localDiff is rect->circle, but we need circle->rect (A->B)
    info.contactPoint = worldClosest;
    return true;
}

bool PhysicsWorld::detectRectRect(const RigidBody& a, const RigidBody& b, CollisionInfo& info)
{
    // Separating Axis Theorem (SAT) for rotated rectangles
    // Test 4 axes: 2 from each rectangle
    
    float cA = std::cos(a.rotation);
    float sA = std::sin(a.rotation);
    float cB = std::cos(b.rotation);
    float sB = std::sin(b.rotation);
    
    Vec2 axes[4];
    axes[0] = Vec2(cA, sA);   // A's x-axis
    axes[1] = Vec2(-sA, cA);  // A's y-axis
    axes[2] = Vec2(cB, sB);   // B's x-axis
    axes[3] = Vec2(-sB, cB);  // B's y-axis
    
    float minPenetration = 1e10f;
    Vec2  minAxis;
    int   minAxisIndex = -1;
    
    for (int i = 0; i < 4; ++i)
    {
        Vec2 axis = axes[i];
        
        // Project both rectangles onto this axis
        // A's corners
        Vec2 aCorners[4];
        aCorners[0] = Vec2(-a.halfSize.x, -a.halfSize.y);
        aCorners[1] = Vec2(a.halfSize.x, -a.halfSize.y);
        aCorners[2] = Vec2(a.halfSize.x, a.halfSize.y);
        aCorners[3] = Vec2(-a.halfSize.x, a.halfSize.y);
        
        float aMin = 1e10f, aMax = -1e10f;
        for (int j = 0; j < 4; ++j)
        {
            Vec2 worldCorner;
            worldCorner.x = a.position.x + aCorners[j].x * cA - aCorners[j].y * sA;
            worldCorner.y = a.position.y + aCorners[j].x * sA + aCorners[j].y * cA;
            float proj = dot(worldCorner, axis);
            aMin = std::min(aMin, proj);
            aMax = std::max(aMax, proj);
        }
        
        // B's corners
        Vec2 bCorners[4];
        bCorners[0] = Vec2(-b.halfSize.x, -b.halfSize.y);
        bCorners[1] = Vec2(b.halfSize.x, -b.halfSize.y);
        bCorners[2] = Vec2(b.halfSize.x, b.halfSize.y);
        bCorners[3] = Vec2(-b.halfSize.x, b.halfSize.y);
        
        float bMin = 1e10f, bMax = -1e10f;
        for (int j = 0; j < 4; ++j)
        {
            Vec2 worldCorner;
            worldCorner.x = b.position.x + bCorners[j].x * cB - bCorners[j].y * sB;
            worldCorner.y = b.position.y + bCorners[j].x * sB + bCorners[j].y * cB;
            float proj = dot(worldCorner, axis);
            bMin = std::min(bMin, proj);
            bMax = std::max(bMax, proj);
        }
        
        // Check for separation
        if (aMax < bMin || bMax < aMin)
            return false;  // Separating axis found
        
        // Calculate penetration
        float penetration = std::min(aMax - bMin, bMax - aMin);
        if (penetration < minPenetration)
        {
            minPenetration = penetration;
            minAxis = axis;
            minAxisIndex = i;
        }
    }
    
    // Ensure normal points from A to B
    Vec2 diff = b.position - a.position;
    if (dot(diff, minAxis) < 0.f)
        minAxis = minAxis * -1.f;
    
    info.hasCollision = true;
    info.normal = minAxis;
    info.penetration = minPenetration;
    
    // Find contact point (approximate: midpoint of penetrating edge)
    // This is a simplified contact point calculation
    Vec2 contactOnA = a.position + minAxis * (a.halfSize.x * std::abs(dot(axes[0], minAxis)) + 
                                               a.halfSize.y * std::abs(dot(axes[1], minAxis)));
    Vec2 contactOnB = b.position - minAxis * (b.halfSize.x * std::abs(dot(axes[2], minAxis)) + 
                                               b.halfSize.y * std::abs(dot(axes[3], minAxis)));
    info.contactPoint = (contactOnA + contactOnB) * 0.5f;
    
    return true;
}

bool PhysicsWorld::detectBodyGround(const RigidBody& body, CollisionInfo& info)
{
    // Ground is an infinite horizontal plane at y = groundY
    // y-down coordinate system: groundY is the TOP edge of the ground
    
    if (body.shape == RigidBody::Circle)
    {
        float bottom = body.position.y + body.radius;
        if (bottom > groundY)
        {
            info.hasCollision = true;
            info.normal = Vec2(0.f, -1.f);  // Normal points upward
            info.penetration = bottom - groundY;
            info.contactPoint = Vec2(body.position.x, groundY);
            return true;
        }
    }
    else // Rect
    {
        // Check all 4 corners
        float c = std::cos(body.rotation);
        float s = std::sin(body.rotation);
        
        float maxPenetration = 0.f;
        float supportXSum = 0.f;
        int supportCount = 0;
        
        Vec2 corners[4];
        corners[0] = Vec2(-body.halfSize.x, -body.halfSize.y);
        corners[1] = Vec2(body.halfSize.x, -body.halfSize.y);
        corners[2] = Vec2(body.halfSize.x, body.halfSize.y);
        corners[3] = Vec2(-body.halfSize.x, body.halfSize.y);
        
        for (int i = 0; i < 4; ++i)
        {
            Vec2 worldCorner;
            worldCorner.x = body.position.x + corners[i].x * c - corners[i].y * s;
            worldCorner.y = body.position.y + corners[i].x * s + corners[i].y * c;
            
            if (worldCorner.y > groundY)
            {
                float pen = worldCorner.y - groundY;
                supportXSum += worldCorner.x;
                ++supportCount;
                if (pen > maxPenetration)
                {
                    maxPenetration = pen;
                }
            }
        }
        
        if (maxPenetration > 0.f && supportCount > 0)
        {
            info.hasCollision = true;
            info.normal = Vec2(0.f, -1.f);
            info.penetration = maxPenetration;
            info.contactPoint = Vec2(supportXSum / static_cast<float>(supportCount), groundY);
            return true;
        }
    }
    
    return false;
}

// ──── Collision Response ────

void PhysicsWorld::resolveCollision(RigidBody& a, RigidBody& b, const CollisionInfo& info)
{
    if (!info.hasCollision)
        return;
    
    if (a.isStatic && b.isStatic)
        return;
    
    Vec2 n = info.normal;
    Vec2 contact = info.contactPoint;
    
    // Relative velocity at contact point
    Vec2 rA = contact - a.position;
    Vec2 rB = contact - b.position;
    
    Vec2 velA = a.velocity + cross2D(a.angularVel, rA);
    Vec2 velB = b.velocity + cross2D(b.angularVel, rB);
    Vec2 relVel = velB - velA;
    
    float velAlongN = dot(relVel, n);
    
    // Objects are separating, skip
    if (velAlongN > 0.f)
        return;
    
    // Combined restitution (use minimum for stability)
    float e = std::min(a.restitution, b.restitution);
    
    // Combined friction (geometric mean)
    float mu = std::sqrt(a.friction * b.friction);
    
    // Normal impulse scalar
    float invMassSum = a.invMass + b.invMass;
    float invInertiaSum = cross2D(rA, n) * cross2D(rA, n) * a.invInertia +
                          cross2D(rB, n) * cross2D(rB, n) * b.invInertia;
    
    float j = -(1.f + e) * velAlongN / (invMassSum + invInertiaSum);
    
    // Apply normal impulse
    Vec2 impulse = n * j;
    
    if (!a.isStatic)
    {
        a.velocity -= impulse * a.invMass;
        a.angularVel -= cross2D(rA, impulse) * a.invInertia;
    }
    
    if (!b.isStatic)
    {
        b.velocity += impulse * b.invMass;
        b.angularVel += cross2D(rB, impulse) * b.invInertia;
    }
    
    // ──── Friction impulse (Coulomb model) ────
    
    // Recalculate relative velocity after normal impulse
    velA = a.velocity + cross2D(a.angularVel, rA);
    velB = b.velocity + cross2D(b.angularVel, rB);
    relVel = velB - velA;
    
    // Tangent direction
    Vec2 tangent = relVel - n * dot(relVel, n);
    float tanLen = tangent.length();
    if (tanLen < 1e-6f)
        return;
    
    tangent = tangent / tanLen;
    float velAlongT = dot(relVel, tangent);
    
    // Friction impulse scalar
    float invInertiaT = cross2D(rA, tangent) * cross2D(rA, tangent) * a.invInertia +
                        cross2D(rB, tangent) * cross2D(rB, tangent) * b.invInertia;
    
    float jt = -velAlongT / (invMassSum + invInertiaT);
    
    // Coulomb friction constraint
    if (std::abs(jt) > mu * std::abs(j))
    {
        jt = -mu * std::abs(j) * (velAlongT > 0.f ? 1.f : -1.f);
    }
    
    // Apply friction impulse
    Vec2 frictionImpulse = tangent * jt;
    
    if (!a.isStatic)
    {
        a.velocity -= frictionImpulse * a.invMass;
        a.angularVel -= cross2D(rA, frictionImpulse) * a.invInertia;
    }
    
    if (!b.isStatic)
    {
        b.velocity += frictionImpulse * b.invMass;
        b.angularVel += cross2D(rB, frictionImpulse) * b.invInertia;
    }
}

void PhysicsWorld::resolveBodyGround(RigidBody& body, const CollisionInfo& info)
{
    if (!info.hasCollision || body.isStatic)
        return;
    
    Vec2 n = info.normal;  // Points upward (-y direction)
    Vec2 contact = info.contactPoint;
    
    // Relative velocity at contact
    Vec2 rA = contact - body.position;
    Vec2 velA = body.velocity + cross2D(body.angularVel, rA);
    
    float velAlongN = dot(velA, n);
    
    // Separating
    if (velAlongN > 0.f)
        return;
    
    // Ground restitution and friction
    float e = body.restitution * 0.8f;  // Ground absorbs more energy
    float mu = std::sqrt(body.friction * 0.6f);  // Ground friction ~0.6
    
    // Normal impulse (ground has infinite mass/inertia)
    float invInertiaSum = cross2D(rA, n) * cross2D(rA, n) * body.invInertia;
    float j = -(1.f + e) * velAlongN / (body.invMass + invInertiaSum);
    
    // Apply normal impulse
    Vec2 impulse = n * j;
    body.velocity += impulse * body.invMass;
    body.angularVel += cross2D(rA, impulse) * body.invInertia;
    
    // Friction impulse
    Vec2 velAfter = body.velocity + cross2D(body.angularVel, rA);
    Vec2 tangent = velAfter - n * dot(velAfter, n);
    float tanLen = tangent.length();
    
    if (tanLen > 1e-6f)
    {
        tangent = tangent / tanLen;
        float velAlongT = dot(velAfter, tangent);
        float invInertiaT = cross2D(rA, tangent) * cross2D(rA, tangent) * body.invInertia;
        float jt = -velAlongT / (body.invMass + invInertiaT);
        
        // Coulomb constraint
        if (std::abs(jt) > mu * std::abs(j))
        {
            jt = -mu * std::abs(j) * (velAlongT > 0.f ? 1.f : -1.f);
        }
        
        Vec2 frictionImpulse = tangent * jt;
        body.velocity += frictionImpulse * body.invMass;
        body.angularVel += cross2D(rA, frictionImpulse) * body.invInertia;
    }

    // Extra rolling resistance for the bird: keep air damping unchanged and
    // only shorten ground roll distance for circles after contact is resolved.
    if (body.shape == RigidBody::Circle)
    {
        body.velocity.x *= 0.985f;
        // Slightly lighter roll damping so rolling stays visible on ground
        body.angularVel *= 0.985f;
    }
}

// ──── Position Correction (Baumgarte) ────

void PhysicsWorld::correctPositions(RigidBody& a, RigidBody& b, const CollisionInfo& info)
{
    if (!info.hasCollision)
        return;
    
    if (a.isStatic && b.isStatic)
        return;
    
    const float slop = 0.5f;    // Penetration allowance (pixels)
    const float percent = 0.6f; // Correction percentage (higher for better separation)
    
    float correction = std::max(info.penetration - slop, 0.f) * percent;
    
    Vec2 correctionVec = info.normal * correction;
    
    float totalInvMass = a.invMass + b.invMass;
    if (totalInvMass < 1e-6f)
        return;
    
    if (!a.isStatic)
        a.position -= correctionVec * (a.invMass / totalInvMass);
    
    if (!b.isStatic)
        b.position += correctionVec * (b.invMass / totalInvMass);
}

void PhysicsWorld::correctPositionGround(RigidBody& body, const CollisionInfo& info)
{
    if (!info.hasCollision || body.isStatic)
        return;
    
    const float slop = 0.5f;
    const float percent = 0.6f;
    
    float correction = std::max(info.penetration - slop, 0.f) * percent;
    body.position.y -= correction;  // Move body upward
}

// ──── Physics Step ────

void PhysicsWorld::step(float dt)
{
    // 1. Integrate velocities and positions
    for (RigidBody& body : bodies_)
    {
        integrate(body, dt);
    }
    
    // 2. Collision detection and response (multiple iterations for stability)
    for (int iter = 0; iter < collisionIterations; ++iter)
    {
        // Body-body collisions
        for (size_t i = 0; i < bodies_.size(); ++i)
        {
            for (size_t j = i + 1; j < bodies_.size(); ++j)
            {
                CollisionInfo info;
                if (detectCollision(bodies_[i], bodies_[j], info))
                {
                    resolveCollision(bodies_[i], bodies_[j], info);
                    correctPositions(bodies_[i], bodies_[j], info);
                }
            }
        }
        
        // Body-ground collisions
        for (RigidBody& body : bodies_)
        {
            CollisionInfo info;
            if (detectBodyGround(body, info))
            {
                resolveBodyGround(body, info);
                correctPositionGround(body, info);
            }
        }
    }

    // 3. Sleep bodies that are resting on the ground or supported by another body.
    for (size_t i = 0; i < bodies_.size(); ++i)
    {
        RigidBody& body = bodies_[i];
        if (body.isStatic)
            continue;

        float velLen = body.velocity.length();
        bool hasSupport = false;

        CollisionInfo groundInfo;
        if (detectBodyGround(body, groundInfo))
        {
            hasSupport = true;
        }
        else
        {
            for (size_t j = 0; j < bodies_.size(); ++j)
            {
                if (i == j)
                    continue;

                CollisionInfo info;
                if (detectCollision(body, bodies_[j], info) && info.normal.y > 0.35f)
                {
                    hasSupport = true;
                    break;
                }
            }
        }

        if (hasSupport && velLen < 5.f && std::abs(body.angularVel) < 0.05f)
        {
            body.velocity = Vec2(0.f, 0.f);
            body.angularVel = 0.f;
        }
    }
}