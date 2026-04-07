#pragma once

#include <cmath>
#include <vector>

/// Pure 2D renderer: standard C++ and system OpenGL only; no third-party libraries.

// ──── Lightweight math types ────

struct Vec2
{
	float x, y;

	Vec2() : x(0.f), y(0.f) {}
	Vec2(float x_, float y_) : x(x_), y(y_) {}

	Vec2 operator+(Vec2 o) const { return Vec2(x + o.x, y + o.y); }
	Vec2 operator-(Vec2 o) const { return Vec2(x - o.x, y - o.y); }
	Vec2 operator*(float s) const { return Vec2(x * s, y * s); }
	Vec2 operator/(float s) const { return Vec2(x / s, y / s); }
	Vec2 &operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
	Vec2 &operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
	Vec2 &operator*=(float s) { x *= s; y *= s; return *this; }

	float length() const { return std::sqrt(x * x + y * y); }
	Vec2 normalized() const
	{
		float l = length();
		return l > 1e-6f ? Vec2(x / l, y / l) : Vec2();
	}
};

struct Color
{
	float r, g, b, a;

	Color() : r(1.f), g(1.f), b(1.f), a(1.f) {}
	Color(float r_, float g_, float b_, float a_ = 1.f) : r(r_), g(g_), b(b_), a(a_) {}

	const float *ptr() const { return &r; }
};

// Preset colors
namespace Colors
{
	static const Color White  (1.f, 1.f, 1.f, 1.f);
	static const Color Black  (0.f, 0.f, 0.f, 1.f);
	static const Color Red    (0.9f, 0.2f, 0.2f, 1.f);
	static const Color Green  (0.2f, 0.85f, 0.2f, 1.f);
	static const Color Blue   (0.2f, 0.4f, 0.9f, 1.f);
	static const Color Orange (0.95f, 0.55f, 0.1f, 1.f);
	static const Color Cyan   (0.2f, 0.75f, 0.75f, 1.f);
	static const Color Brown  (0.35f, 0.28f, 0.2f, 1.f);
	static const Color Grass  (0.25f, 0.55f, 0.22f, 1.f);
	static const Color Sky    (0.35f, 0.55f, 0.75f, 1.f);

	/// Stable pastel fill color from integer ID (HSL to RGB).
	inline Color pastelFromId(int id)
	{
		unsigned u = static_cast<unsigned>(id) * 2654435761u;
		float hue = static_cast<float>(u % 360u);
		
		// HSL to RGB conversion
		float h = std::fmod(hue, 360.f);
		if (h < 0.f) h += 360.f;
		float s = 0.42f;
		float l = 0.72f;
		float c = (1.f - std::abs(2.f * l - 1.f)) * s;
		float x = c * (1.f - std::abs(std::fmod(h / 60.f, 2.f) - 1.f));
		float m = l - c * 0.5f;
		float rp = 0.f, gp = 0.f, bp = 0.f;
		
		if (h < 60.f)      { rp = c; gp = x; }
		else if (h < 120.f){ rp = x; gp = c; }
		else if (h < 180.f){ gp = c; bp = x; }
		else if (h < 240.f){ gp = x; bp = c; }
		else if (h < 300.f){ rp = x; bp = c; }
		else               { rp = c; bp = x; }
		
		return Color(rp + m, gp + m, bp + m, 1.f);
	}
}

// ──── Particles (simple) ────

struct Particle
{
	Vec2 pos, vel;
	Color color;
	float life;     // Remaining lifetime (seconds)
	float maxLife;
	float size;
};

// ──── Renderer ────

class Renderer2D
{
public:
	void init();
	void shutdown();

	/// Start of frame: viewport, clear (sky blue), orthographic projection.
	void beginFrame(int framebufferW, int framebufferH);

	/// Sky, distant mountains, and clouds behind the ground plane.
	void drawBackground(float floorY, float screenW);

	/// Subsoil + grass strip + top seam line (full width).
	void drawGround(float floorY, float screenW);

	/// Thick line segments (quads; does not rely on glLineWidth).
	void drawLine(Vec2 a, Vec2 b, Color color, float width);

	/// Filled rectangle, optional rotation in radians (about center).
	void drawFilledRect(Vec2 pos, Vec2 size, Color color, float rotationRad = 0.f);

	/// Rectangle outline, optional rotation in radians.
	void drawRectOutline(Vec2 pos, Vec2 size, Color color, float lineWidth, float rotationRad = 0.f);

	/// Filled quad from four corner points.
	void drawFilledQuad(const Vec2 corners[4], Color color);

	/// Quad outline from four corner points.
	void drawQuadOutline(const Vec2 corners[4], Color color, float lineWidth);

	/// Circle outline.
	void drawCircleOutline(Vec2 center, float radius, Color color, float lineWidth, int segments = 48);

	/// Filled circle.
	void drawFilledCircle(Vec2 center, float radius, Color color, int segments = 48);

	/// Bird: body circle + eye + beak; features rotate with rotationRad (physics roll).
	void drawBird(Vec2 center, float radius, Color bodyColor, float rotationRad = 0.f);

	/// Green pig: round body, snout, eyes; face detail rotates with rotationRad.
	void drawPig(Vec2 center, float radius, float rotationRad = 0.f);

	/// Brick: fill + outline + horizontal stripes.
	void drawBrick(Vec2 pos, Vec2 size, Color fillColor, float rotationRad = 0.f);

	/// Slingshot bands (two arms to pull point) + small cradle circle.
	void drawSlingshot(Vec2 anchor, Vec2 pullPoint, float bandWidth = 3.f);

	/// Parabolic trajectory preview (constant acceleration); pass gravity as +y scalar.
	void drawTrajectory(Vec2 launchPos, Vec2 launchVel, float gravity,
	                    int dotCount = 30, float timeStep = 0.05f);

	/// Spawn random-direction particles near origin (total count capped).
	void spawnParticles(Vec2 origin, int count, Color baseColor);

	/// Update, draw, and remove dead particles.
	void updateAndDrawParticles(float deltaTime);

	/// Semi-transparent pause overlay with a simple PAUSED label.
	void drawPausedOverlay(float screenW, float screenH);

	/// Debug marker: contact point + normal.
	void drawContactDebug(Vec2 point, Vec2 normal, bool visible);

	/// End of frame (reserved; no-op now; batching could flush here later).
	void flush();

private:
	void ensureGlState();
	void updateProjection(int w, int h);
	void uploadAndDraw(const float *xy, int vertCount, Color color);
	void thickLine(Vec2 a, Vec2 b, Color color, float width);

	/// Four corners of the rect at pos/size after rotation by rad about its center.
	static void rectCorners(Vec2 pos, Vec2 size, float rad, Vec2 out[4]);

	unsigned int program_  = 0;
	int uMvp_     = -1;
	int uColor_   = -1;
	unsigned int vao_      = 0;
	unsigned int vbo_      = 0;

	float proj_[16] = {};
	bool initialized_ = false;

	std::vector<Particle> particles_;
	static const int kMaxParticles = 200;
};
