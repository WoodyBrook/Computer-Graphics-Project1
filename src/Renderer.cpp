#include "Renderer.h"

#define GL_SILENCE_DEPRECATION
#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// ──── Internal helpers ────

namespace
{

namespace fs = std::filesystem;

const float kPi    = 3.14159265358979323846f;
const float kTwoPi = 2.f * kPi;

float vec2Length(Vec2 v)
{
	return std::sqrt(v.x * v.x + v.y * v.y);
}

Vec2 vec2Perp(Vec2 v)
{
	return Vec2(-v.y, v.x);
}

Vec2 rotatePoint(Vec2 p, Vec2 center, float rad)
{
	float c = std::cos(rad);
	float s = std::sin(rad);
	float dx = p.x - center.x;
	float dy = p.y - center.y;
	return Vec2(center.x + dx * c - dy * s,
	            center.y + dx * s + dy * c);
}

void ortho(float l, float r, float b, float t, float n, float f, float out[16])
{
	std::memset(out, 0, 16 * sizeof(float));
	out[0]  =  2.f / (r - l);
	out[5]  =  2.f / (t - b);
	out[10] = -2.f / (f - n);
	out[12] = -(r + l) / (r - l);
	out[13] = -(t + b) / (t - b);
	out[14] = -(f + n) / (f - n);
	out[15] =  1.f;
}

GLuint compileShader(GLenum type, const char *src)
{
	GLuint s = glCreateShader(type);
	glShaderSource(s, 1, &src, nullptr);
	glCompileShader(s);
	GLint ok = 0;
	glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
	if (!ok)
	{
		char buf[1024];
		glGetShaderInfoLog(s, sizeof(buf), nullptr, buf);
		std::cerr << "Shader compile error:\n" << buf << "\n";
		glDeleteShader(s);
		return 0;
	}
	return s;
}

GLuint linkProgram(GLuint vs, GLuint fs)
{
	GLuint p = glCreateProgram();
	glAttachShader(p, vs);
	glAttachShader(p, fs);
	glLinkProgram(p);
	GLint ok = 0;
	glGetProgramiv(p, GL_LINK_STATUS, &ok);
	if (!ok)
	{
		char buf[1024];
		glGetProgramInfoLog(p, sizeof(buf), nullptr, buf);
		std::cerr << "Program link error:\n" << buf << "\n";
		glDeleteProgram(p);
		return 0;
	}
	return p;
}

float clamp01(float x)
{
	return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}

Color withAlpha(Color c, float a)
{
	return Color(c.r, c.g, c.b, a);
}

Color lighten(Color c, float amount)
{
	return Color(
		clamp01(c.r + amount),
		clamp01(c.g + amount),
		clamp01(c.b + amount),
		c.a);
}

Color darken(Color c, float amount)
{
	return Color(
		clamp01(c.r - amount),
		clamp01(c.g - amount),
		clamp01(c.b - amount),
		c.a);
}

Vec2 lerp(Vec2 a, Vec2 b, float t)
{
	return a + (b - a) * t;
}

Color hslToRgb(float hDeg, float s, float l)
{
	float h = std::fmod(hDeg, 360.f);
	if (h < 0.f)
		h += 360.f;
	float c = (1.f - std::abs(2.f * l - 1.f)) * s;
	float x = c * (1.f - std::abs(std::fmod(h / 60.f, 2.f) - 1.f));
	float m = l - c * 0.5f;
	float rp = 0.f, gp = 0.f, bp = 0.f;
	if (h < 60.f)
	{
		rp = c;
		gp = x;
	}
	else if (h < 120.f)
	{
		rp = x;
		gp = c;
	}
	else if (h < 180.f)
	{
		gp = c;
		bp = x;
	}
	else if (h < 240.f)
	{
		gp = x;
		bp = c;
	}
	else if (h < 300.f)
	{
		rp = x;
		bp = c;
	}
	else
	{
		rp = c;
		bp = x;
	}
	return Color(rp + m, gp + m, bp + m, 1.f);
}

std::string resolveAssetPath(const char *relativePath)
{
	fs::path candidate(relativePath);
	if (fs::exists(candidate))
		return candidate.string();

#ifdef ANGRY_BIRD_SOURCE_DIR
	candidate = fs::path(ANGRY_BIRD_SOURCE_DIR) / relativePath;
	if (fs::exists(candidate))
		return candidate.string();
#endif

	return relativePath;
}

bool loadTextureFromFile(const char *relativePath, GLuint &textureId, int &width, int &height,
                         bool removeWhiteBackground)
{
	const std::string path = resolveAssetPath(relativePath);
	stbi_set_flip_vertically_on_load(1);
	int channels = 0;
	unsigned char *pixels = stbi_load(path.c_str(), &width, &height, &channels, STBI_rgb_alpha);
	if (!pixels)
	{
		std::cerr << "Failed to load texture: " << path << " (" << stbi_failure_reason() << ")\n";
		return false;
	}

	if (removeWhiteBackground)
	{
		auto isNearWhite = [&](int pixelIndex) -> bool
		{
			const int base = pixelIndex * 4;
			const int r = pixels[base + 0];
			const int g = pixels[base + 1];
			const int b = pixels[base + 2];
			const int minCh = std::min(r, std::min(g, b));
			const int maxCh = std::max(r, std::max(g, b));
			return minCh >= 236 && (maxCh - minCh) <= 28;
		};

		std::vector<unsigned char> borderVisited(static_cast<size_t>(width * height), 0);
		std::vector<int> queue;
		queue.reserve(static_cast<size_t>(width + height) * 2u);

		auto enqueueBorderWhite = [&](int x, int y)
		{
			if (x < 0 || y < 0 || x >= width || y >= height)
				return;
			const int idx = y * width + x;
			if (borderVisited[static_cast<size_t>(idx)] || !isNearWhite(idx))
				return;
			borderVisited[static_cast<size_t>(idx)] = 1;
			queue.push_back(idx);
		};

		for (int x = 0; x < width; ++x)
		{
			enqueueBorderWhite(x, 0);
			enqueueBorderWhite(x, height - 1);
		}
		for (int y = 0; y < height; ++y)
		{
			enqueueBorderWhite(0, y);
			enqueueBorderWhite(width - 1, y);
		}

		for (size_t head = 0; head < queue.size(); ++head)
		{
			const int idx = queue[head];
			const int x = idx % width;
			const int y = idx / width;
			const int neighbors[4][2] = {
				{x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1}};
			for (const auto &neighbor : neighbors)
			{
				const int nx = neighbor[0];
				const int ny = neighbor[1];
				if (nx < 0 || ny < 0 || nx >= width || ny >= height)
					continue;
				const int nidx = ny * width + nx;
				if (borderVisited[static_cast<size_t>(nidx)] || !isNearWhite(nidx))
					continue;
				borderVisited[static_cast<size_t>(nidx)] = 1;
				queue.push_back(nidx);
			}
		}

		for (size_t i = 0; i < borderVisited.size(); ++i)
		{
			if (!borderVisited[i])
				continue;
			pixels[i * 4 + 3] = 0;
		}
	}

	glGenTextures(1, &textureId);
	glBindTexture(GL_TEXTURE_2D, textureId);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	glBindTexture(GL_TEXTURE_2D, 0);
	stbi_image_free(pixels);
	return true;
}

} // namespace

// ──── Internal methods ────

void Renderer2D::ensureGlState()
{
	glEnable(GL_BLEND);
	glBlendEquation(GL_FUNC_ADD);
	glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
}

void Renderer2D::updateProjection(int w, int h)
{
	ortho(0.f, static_cast<float>(w), static_cast<float>(h), 0.f, -1.f, 1.f, proj_);
}

void Renderer2D::uploadAndDraw(const float *xy, int vertCount, Color color)
{
	if (vertCount < 3)
		return;
	glUseProgram(program_);
	glUniformMatrix4fv(uMvp_, 1, GL_FALSE, proj_);
	float c4[4] = {color.r, color.g, color.b, color.a};
	glUniform4fv(uColor_, 1, c4);

	glBindVertexArray(vao_);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_);
	glBufferData(GL_ARRAY_BUFFER,
	             static_cast<GLsizeiptr>(vertCount * 2 * sizeof(float)),
	             xy, GL_STREAM_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);

	glDrawArrays(GL_TRIANGLES, 0, vertCount);
}

void Renderer2D::uploadAndDrawTextured(const float *xyuv, int vertCount, unsigned int textureId,
                                       float whiteKeyThreshold, float whiteKeySoftness)
{
	if (vertCount < 3 || textureId == 0 || texturedProgram_ == 0)
		return;

	glUseProgram(texturedProgram_);
	glUniformMatrix4fv(uTexMvp_, 1, GL_FALSE, proj_);
	glUniform1f(uWhiteKeyThreshold_, whiteKeyThreshold);
	glUniform1f(uWhiteKeySoftness_, whiteKeySoftness);
	glUniform1i(uSampler_, 0);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureId);
	glBindVertexArray(vao_);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_);
	glBufferData(GL_ARRAY_BUFFER,
	             static_cast<GLsizeiptr>(vertCount * 4 * sizeof(float)),
	             xyuv, GL_STREAM_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
	                      reinterpret_cast<const void *>(2 * sizeof(float)));

	glDrawArrays(GL_TRIANGLES, 0, vertCount);
	glBindTexture(GL_TEXTURE_2D, 0);
}

void Renderer2D::thickLine(Vec2 a, Vec2 b, Color color, float width)
{
	Vec2 d = b - a;
	float len = vec2Length(d);
	if (len < 1e-6f)
		return;
	d = d / len;
	Vec2 n = vec2Perp(d);
	Vec2 o = n * (width * 0.5f);
	Vec2 p0 = a + o, p1 = a - o, p2 = b - o, p3 = b + o;
	const float v[] = {
		p0.x, p0.y, p1.x, p1.y, p2.x, p2.y,
		p0.x, p0.y, p2.x, p2.y, p3.x, p3.y};
	uploadAndDraw(v, 6, color);
}

void Renderer2D::destroyTexture(unsigned int &textureId)
{
	if (textureId)
		glDeleteTextures(1, &textureId);
	textureId = 0;
}

void Renderer2D::rectCorners(Vec2 pos, Vec2 size, float rad, Vec2 out[4])
{
	out[0] = pos;
	out[1] = Vec2(pos.x, pos.y + size.y);
	out[2] = Vec2(pos.x + size.x, pos.y + size.y);
	out[3] = Vec2(pos.x + size.x, pos.y);

	if (rad != 0.f)
	{
		Vec2 center(pos.x + size.x * 0.5f, pos.y + size.y * 0.5f);
		for (int i = 0; i < 4; ++i)
			out[i] = rotatePoint(out[i], center, rad);
	}
}

// ──── Public API ────

void Renderer2D::init()
{
	if (initialized_)
		return;

	ensureGlState();

	const char *vsSrc = R"(#version 330 core
layout (location = 0) in vec2 aPos;
uniform mat4 u_mvp;
void main() { gl_Position = u_mvp * vec4(aPos, 0.0, 1.0); }
)";
	const char *fsSrc = R"(#version 330 core
layout (location = 0) out vec4 fragColor;
uniform vec4 u_color;
void main() { fragColor = u_color; }
)";
	const char *texVsSrc = R"(#version 330 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec2 aUv;
uniform mat4 u_mvp;
out vec2 v_uv;
void main()
{
	v_uv = aUv;
	gl_Position = u_mvp * vec4(aPos, 0.0, 1.0);
}
)";
	const char *texFsSrc = R"(#version 330 core
in vec2 v_uv;
layout (location = 0) out vec4 fragColor;
uniform sampler2D u_tex;
uniform float u_whiteKeyThreshold;
uniform float u_whiteKeySoftness;
void main()
{
	vec4 texel = texture(u_tex, v_uv);
	if (texel.a < 0.02)
		discard;
	fragColor = texel;
}
)";

	GLuint vs = compileShader(GL_VERTEX_SHADER, vsSrc);
	GLuint fs = compileShader(GL_FRAGMENT_SHADER, fsSrc);
	program_ = linkProgram(vs, fs);
	glDeleteShader(vs);
	glDeleteShader(fs);

	GLuint texVs = compileShader(GL_VERTEX_SHADER, texVsSrc);
	GLuint texFs = compileShader(GL_FRAGMENT_SHADER, texFsSrc);
	texturedProgram_ = linkProgram(texVs, texFs);
	glDeleteShader(texVs);
	glDeleteShader(texFs);

	uMvp_   = glGetUniformLocation(program_, "u_mvp");
	uColor_ = glGetUniformLocation(program_, "u_color");
	uTexMvp_ = glGetUniformLocation(texturedProgram_, "u_mvp");
	uSampler_ = glGetUniformLocation(texturedProgram_, "u_tex");
	uWhiteKeyThreshold_ = glGetUniformLocation(texturedProgram_, "u_whiteKeyThreshold");
	uWhiteKeySoftness_ = glGetUniformLocation(texturedProgram_, "u_whiteKeySoftness");

	glGenVertexArrays(1, &vao_);
	glGenBuffers(1, &vbo_);

	loadTextureFromFile("assets/textures/bird.png", birdTexture_, birdTextureW_, birdTextureH_, true);
	loadTextureFromFile("assets/textures/pig.png", pigTexture_, pigTextureW_, pigTextureH_, true);
	loadTextureFromFile("assets/textures/background.png", backgroundTexture_,
	                    backgroundTextureW_, backgroundTextureH_, false);

	initialized_ = true;
}

void Renderer2D::shutdown()
{
	if (!initialized_)
		return;
	destroyTexture(birdTexture_);
	destroyTexture(pigTexture_);
	destroyTexture(backgroundTexture_);
	if (vbo_)     glDeleteBuffers(1, &vbo_);
	if (vao_)     glDeleteVertexArrays(1, &vao_);
	if (texturedProgram_) glDeleteProgram(texturedProgram_);
	if (program_) glDeleteProgram(program_);
	birdTextureW_ = birdTextureH_ = 0;
	pigTextureW_ = pigTextureH_ = 0;
	backgroundTextureW_ = backgroundTextureH_ = 0;
	vbo_ = vao_ = program_ = texturedProgram_ = 0;
	initialized_ = false;
}

void Renderer2D::beginFrame(int framebufferW, int framebufferH)
{
	updateProjection(framebufferW, framebufferH);
	glViewport(0, 0, framebufferW, framebufferH);
	glClearColor(Colors::Sky.r, Colors::Sky.g, Colors::Sky.b, Colors::Sky.a);
	glClear(GL_COLOR_BUFFER_BIT);
	ensureGlState();
}

void Renderer2D::drawBackground(float floorY, float screenW)
{
	if (backgroundTexture_ != 0 && backgroundTextureW_ > 0 && backgroundTextureH_ > 0)
	{
		const float targetW = screenW;
		const float targetH = floorY;
		const float texAspect = static_cast<float>(backgroundTextureW_) /
		                        static_cast<float>(backgroundTextureH_);
		const float targetAspect = targetW / std::max(targetH, 1.f);

		float uMin = 0.f, uMax = 1.f;
		float vMin = 0.f, vMax = 1.f;
		if (texAspect > targetAspect)
		{
			const float visibleU = targetAspect / texAspect;
			const float margin = (1.f - visibleU) * 0.5f;
			uMin = margin;
			uMax = 1.f - margin;
		}
		else if (texAspect < targetAspect)
		{
			const float visibleV = texAspect / targetAspect;
			const float margin = (1.f - visibleV) * 0.5f;
			vMin = margin;
			vMax = 1.f - margin;
		}

		const Vec2 corners[4] = {
			Vec2(0.f, 0.f),
			Vec2(0.f, targetH),
			Vec2(targetW, targetH),
			Vec2(targetW, 0.f)};
		const float v[] = {
			corners[0].x, corners[0].y, uMin, vMax,
			corners[1].x, corners[1].y, uMin, vMin,
			corners[2].x, corners[2].y, uMax, vMin,
			corners[0].x, corners[0].y, uMin, vMax,
			corners[2].x, corners[2].y, uMax, vMin,
			corners[3].x, corners[3].y, uMax, vMax};
		uploadAndDrawTextured(v, 6, backgroundTexture_, 0.f, 0.f);
		return;
	}

	// Sky bands: gradient from deep blue at top to lighter blue near horizon
	const float coverW = std::max(screenW, 1400.f);
	const float bandH = floorY / 4.f;
	
	Color skyDeep(0.15f, 0.25f, 0.45f, 1.f);
	Color skyMid(0.22f, 0.38f, 0.58f, 1.f);
	Color skyLight(0.32f, 0.52f, 0.72f, 1.f);
	Color skyHorizon(0.45f, 0.62f, 0.78f, 1.f);
	
	// Draw 4 horizontal bands from top to bottom
	drawFilledRect(Vec2(0.f, 0.f), Vec2(coverW, bandH), skyDeep);
	drawFilledRect(Vec2(0.f, bandH), Vec2(coverW, bandH), skyMid);
	drawFilledRect(Vec2(0.f, bandH * 2.f), Vec2(coverW, bandH), skyLight);
	drawFilledRect(Vec2(0.f, bandH * 3.f), Vec2(coverW, bandH), skyHorizon);
	
	// Far mountain layer (lighter, more distant)
	Color farMountain(0.55f, 0.58f, 0.62f, 0.7f);
	float farY = floorY - 80.f;
	drawFilledCircle(Vec2(200.f, farY), 120.f, farMountain, 32);
	drawFilledCircle(Vec2(450.f, farY + 20.f), 100.f, farMountain, 32);
	drawFilledCircle(Vec2(700.f, farY), 140.f, farMountain, 32);
	drawFilledCircle(Vec2(1000.f, farY + 15.f), 110.f, farMountain, 32);
	drawFilledRect(Vec2(0.f, farY), Vec2(coverW, 200.f), farMountain);
	
	// Near mountain layer (slightly darker, closer)
	Color nearMountain(0.42f, 0.48f, 0.52f, 0.85f);
	float nearY = floorY - 40.f;
	drawFilledCircle(Vec2(150.f, nearY), 90.f, nearMountain, 32);
	drawFilledCircle(Vec2(400.f, nearY + 30.f), 70.f, nearMountain, 32);
	drawFilledCircle(Vec2(650.f, nearY), 100.f, nearMountain, 32);
	drawFilledCircle(Vec2(900.f, nearY + 25.f), 80.f, nearMountain, 32);
	drawFilledRect(Vec2(0.f, nearY), Vec2(coverW, 150.f), nearMountain);
	
	// Clouds: 3 circles + 1 rectangle per cloud
	Color cloudColor = withAlpha(Colors::White, 0.18f);
	Color cloudBright = withAlpha(Colors::White, 0.22f);
	
	// Cloud 1
	Vec2 cloud1Center(300.f, 60.f);
	drawFilledCircle(cloud1Center, 35.f, cloudColor, 24);
	drawFilledCircle(cloud1Center + Vec2(-30.f, 10.f), 28.f, cloudColor, 24);
	drawFilledCircle(cloud1Center + Vec2(32.f, 8.f), 30.f, cloudBright, 24);
	drawFilledRect(cloud1Center + Vec2(-30.f, 10.f), Vec2(65.f, 20.f), cloudColor);
	
	// Cloud 2
	Vec2 cloud2Center(700.f, 100.f);
	drawFilledCircle(cloud2Center, 40.f, cloudBright, 24);
	drawFilledCircle(cloud2Center + Vec2(-35.f, 12.f), 32.f, cloudColor, 24);
	drawFilledCircle(cloud2Center + Vec2(38.f, 10.f), 34.f, cloudColor, 24);
	drawFilledRect(cloud2Center + Vec2(-35.f, 12.f), Vec2(75.f, 22.f), cloudColor);
	
	// Cloud 3
	Vec2 cloud3Center(1100.f, 50.f);
	drawFilledCircle(cloud3Center, 30.f, cloudColor, 24);
	drawFilledCircle(cloud3Center + Vec2(-25.f, 8.f), 24.f, cloudColor, 24);
	drawFilledCircle(cloud3Center + Vec2(28.f, 6.f), 26.f, cloudBright, 24);
	drawFilledRect(cloud3Center + Vec2(-25.f, 8.f), Vec2(55.f, 18.f), cloudColor);
}

void Renderer2D::drawGround(float floorY, float screenW)
{
	// Grass color hierarchy
	const Color grassBright(0.35f, 0.65f, 0.28f, 1.f);  // Top edge highlight
	const Color grassMain(0.25f, 0.55f, 0.22f, 1.f);    // Main grass color
	const Color grassDark(0.18f, 0.38f, 0.15f, 1.f);    // Bottom edge shadow
	const Color grassMid(0.28f, 0.58f, 0.24f, 1.f);     // Internal bright line
	
	// Dirt color hierarchy
	const Color dirtMain(0.42f, 0.32f, 0.22f, 1.f);
	const Color dirtBand1(0.38f, 0.28f, 0.18f, 1.f);
	const Color dirtBand2(0.48f, 0.36f, 0.26f, 1.f);
	
	// Seam color
	const Color seam(0.15f, 0.25f, 0.12f, 1.f);
	
	const float grassH = 48.f;
	const float dirtDepth = 500.f;
	
	// Dirt layer: from bottom of grass to deep underground
	Vec2 dirtQuad[4] = {
		Vec2(0.f, floorY + grassH),
		Vec2(0.f, floorY + grassH + dirtDepth),
		Vec2(screenW, floorY + grassH + dirtDepth),
		Vec2(screenW, floorY + grassH)};
	drawFilledQuad(dirtQuad, dirtMain);
	
	// Dirt horizontal bands for texture
	float bandY1 = floorY + grassH + 80.f;
	float bandY2 = floorY + grassH + 180.f;
	drawFilledRect(Vec2(0.f, bandY1), Vec2(screenW, 25.f), dirtBand1);
	drawFilledRect(Vec2(0.f, bandY2), Vec2(screenW, 20.f), dirtBand2);
	
	// Grass layer: main strip
	Vec2 grassQuad[4] = {
		Vec2(0.f, floorY),
		Vec2(0.f, floorY + grassH),
		Vec2(screenW, floorY + grassH),
		Vec2(screenW, floorY)};
	drawFilledQuad(grassQuad, grassMain);
	
	// Grass top bright edge
	drawFilledRect(Vec2(0.f, floorY), Vec2(screenW, 6.f), grassBright);
	
	// Grass internal bright line
	float midLineY = floorY + grassH * 0.35f;
	drawFilledRect(Vec2(0.f, midLineY), Vec2(screenW, 3.f), grassMid);
	
	// Grass bottom dark edge (seam)
	thickLine(Vec2(0.f, floorY + grassH), Vec2(screenW, floorY + grassH), seam, 4.f);
	thickLine(Vec2(0.f, floorY + grassH - 2.f), Vec2(screenW, floorY + grassH - 2.f), grassDark, 3.f);
}

void Renderer2D::drawBird(Vec2 center, float radius, Color bodyColor, float rotationRad)
{
	(void)bodyColor;
	if (birdTexture_ != 0 && birdTextureW_ > 0 && birdTextureH_ > 0)
	{
		auto rw = [&](Vec2 offsetFromCenter) -> Vec2
		{
			return rotatePoint(center + offsetFromCenter, center, rotationRad);
		};

		const float spriteW = radius * 2.85f;
		const float spriteH = spriteW * static_cast<float>(birdTextureH_) /
		                     static_cast<float>(birdTextureW_);
		const Vec2 spriteCenter = rw(Vec2(-radius * 0.10f, -radius * 0.16f));
		const Vec2 spritePos(spriteCenter.x - spriteW * 0.5f, spriteCenter.y - spriteH * 0.5f);
		Vec2 corners[4];
		rectCorners(spritePos, Vec2(spriteW, spriteH), rotationRad, corners);
		const float v[] = {
			corners[0].x, corners[0].y, 0.f, 1.f,
			corners[1].x, corners[1].y, 0.f, 0.f,
			corners[2].x, corners[2].y, 1.f, 0.f,
			corners[0].x, corners[0].y, 0.f, 1.f,
			corners[2].x, corners[2].y, 1.f, 0.f,
			corners[3].x, corners[3].y, 1.f, 1.f};
		drawFilledCircle(center + Vec2(4.f, 5.f), radius * 1.02f, Color(0.f, 0.f, 0.f, 0.16f), 28);
		uploadAndDrawTextured(v, 6, birdTexture_, 0.f, 0.f);
		return;
	}

	auto rw = [&](Vec2 offsetFromCenter) -> Vec2
	{
		return rotatePoint(center + offsetFromCenter, center, rotationRad);
	};

	// Shadow layer (screen-space drop shadow, not rotated with bird)
	Color shadowColor = darken(bodyColor, 0.35f);
	Vec2 shadowOffset = Vec2(3.f, 4.f);
	drawFilledCircle(center + shadowOffset, radius * 0.95f, shadowColor, 32);
	
	// Main body
	drawFilledCircle(center, radius, bodyColor, 32);
	
	// Highlight
	Color highlightColor = lighten(bodyColor, 0.18f);
	Vec2 highlightPos = rw(Vec2(-radius * 0.35f, -radius * 0.35f));
	drawFilledCircle(highlightPos, radius * 0.38f, highlightColor, 24);
	
	// Outer outline
	Color outlineColor = darken(bodyColor, 0.45f);
	drawCircleOutline(center, radius, outlineColor, 2.f, 32);
	
	// Eye (facing +x in local bird space)
	float eyeR = radius * 0.22f;
	Vec2 eyeLocal = Vec2(radius * 0.32f, -radius * 0.22f);
	Vec2 eyeCenter = rw(eyeLocal);
	
	// Eye outline
	drawCircleOutline(eyeCenter, eyeR + 1.f, darken(Colors::White, 0.2f), 1.5f, 16);
	drawFilledCircle(eyeCenter, eyeR, Colors::White, 16);
	
	// Pupil
	Vec2 pupilLocal = eyeLocal + Vec2(eyeR * 0.28f, 0.f);
	Vec2 pupilCenter = rw(pupilLocal);
	drawFilledCircle(pupilCenter, eyeR * 0.42f, Colors::Black, 12);
	
	// Pupil highlight
	Vec2 pupilHiLocal = pupilLocal + Vec2(-eyeR * 0.12f, -eyeR * 0.12f);
	drawFilledCircle(rw(pupilHiLocal), eyeR * 0.12f, Colors::White, 8);
	
	// Eyebrows
	Color browColor(0.25f, 0.18f, 0.12f, 1.f);
	Vec2 browStart = rw(eyeLocal + Vec2(-eyeR * 0.8f, -eyeR * 0.6f));
	Vec2 browEnd = rw(eyeLocal + Vec2(eyeR * 1.2f, -eyeR * 1.1f));
	thickLine(browStart, browEnd, browColor, 2.5f);
	
	// Beak with outline
	float beak = radius * 0.35f;
	Vec2 tip = rw(Vec2(radius + beak * 0.35f, 0.f));
	Vec2 b1 = rw(Vec2(radius * 0.55f, beak * 0.45f));
	Vec2 b2 = rw(Vec2(radius * 0.55f, -beak * 0.45f));
	Vec2 tri[4] = {tip, b1, b2, b2};
	
	// Beak fill
	Color beakColor(0.95f, 0.55f, 0.1f, 1.f);
	Color beakDark(0.8f, 0.42f, 0.08f, 1.f);
	drawFilledQuad(tri, beakColor);
	
	// Beak outline
	thickLine(tip, b1, beakDark, 1.5f);
	thickLine(tip, b2, beakDark, 1.5f);
	thickLine(b1, b2, beakDark, 1.5f);
}

void Renderer2D::drawPig(Vec2 center, float radius, float rotationRad)
{
	if (pigTexture_ != 0 && pigTextureW_ > 0 && pigTextureH_ > 0)
	{
		const float spriteH = radius * 2.45f;
		const float spriteW = spriteH * static_cast<float>(pigTextureW_) /
		                     static_cast<float>(pigTextureH_);
		const Vec2 spriteCenter = rotatePoint(center + Vec2(0.f, -radius * 0.08f), center, rotationRad);
		const Vec2 spritePos(spriteCenter.x - spriteW * 0.5f, spriteCenter.y - spriteH * 0.5f);
		Vec2 corners[4];
		rectCorners(spritePos, Vec2(spriteW, spriteH), rotationRad, corners);
		const float v[] = {
			corners[0].x, corners[0].y, 0.f, 1.f,
			corners[1].x, corners[1].y, 0.f, 0.f,
			corners[2].x, corners[2].y, 1.f, 0.f,
			corners[0].x, corners[0].y, 0.f, 1.f,
			corners[2].x, corners[2].y, 1.f, 0.f,
			corners[3].x, corners[3].y, 1.f, 1.f};
		drawFilledCircle(center + Vec2(4.f, 5.f), radius * 0.98f, Color(0.f, 0.f, 0.f, 0.14f), 28);
		uploadAndDrawTextured(v, 6, pigTexture_, 0.f, 0.f);
		return;
	}

	auto rw = [&](Vec2 offsetFromCenter) -> Vec2
	{
		return rotatePoint(center + offsetFromCenter, center, rotationRad);
	};

	Color bodyGreen(0.55f, 0.78f, 0.35f, 1.f);
	Color bodyDark(0.32f, 0.52f, 0.22f, 1.f);

	// Shadow (screen-space, not rolled with pig)
	Color shadowColor(0.12f, 0.14f, 0.1f, 0.45f);
	drawFilledCircle(center + Vec2(4.f, 5.f), radius * 0.92f, shadowColor, 28);

	// Body
	drawFilledCircle(center, radius, bodyGreen, 36);
	drawCircleOutline(center, radius, bodyDark, 2.2f, 36);

	// Cheek highlights
	Color cheek = lighten(bodyGreen, 0.12f);
	drawFilledCircle(rw(Vec2(-radius * 0.38f, radius * 0.18f)), radius * 0.22f, cheek, 16);
	drawFilledCircle(rw(Vec2(radius * 0.38f, radius * 0.18f)), radius * 0.22f, cheek, 16);

	// Snout (local +y downward on screen; rotates with body)
	float snR = radius * 0.42f;
	Vec2 snoutLocal = Vec2(radius * 0.08f, radius * 0.32f);
	Vec2 snoutC = rw(snoutLocal);
	Color snoutLight(0.72f, 0.88f, 0.55f, 1.f);
	Color snoutDark(0.38f, 0.55f, 0.28f, 1.f);
	drawFilledCircle(snoutC, snR, snoutLight, 24);
	drawCircleOutline(snoutC, snR, snoutDark, 1.6f, 24);

	// Nostrils
	float nostR = snR * 0.16f;
	drawFilledCircle(rw(snoutLocal + Vec2(-snR * 0.35f, snR * 0.1f)), nostR, snoutDark, 10);
	drawFilledCircle(rw(snoutLocal + Vec2(snR * 0.35f, snR * 0.1f)), nostR, snoutDark, 10);

	// Eyes (cartoon, local +x is pig forward)
	float eyeRad = radius * 0.18f;
	Vec2 eyeLLocal = Vec2(-radius * 0.32f, -radius * 0.15f);
	Vec2 eyeRightLocal = Vec2(radius * 0.32f, -radius * 0.15f);
	Vec2 eyeL = rw(eyeLLocal);
	Vec2 eyeRpos = rw(eyeRightLocal);
	drawCircleOutline(eyeL, eyeRad + 1.f, snoutDark, 1.4f, 16);
	drawFilledCircle(eyeL, eyeRad, Colors::White, 16);
	drawFilledCircle(rw(eyeLLocal + Vec2(eyeRad * 0.25f, 0.f)), eyeRad * 0.45f, Colors::Black, 12);

	drawCircleOutline(eyeRpos, eyeRad + 1.f, snoutDark, 1.4f, 16);
	drawFilledCircle(eyeRpos, eyeRad, Colors::White, 16);
	drawFilledCircle(rw(eyeRightLocal + Vec2(eyeRad * 0.25f, 0.f)), eyeRad * 0.45f, Colors::Black, 12);

	// Thick brow ridge
	thickLine(
	    rw(Vec2(-radius * 0.65f, -radius * 0.48f)),
	    rw(Vec2(radius * 0.65f, -radius * 0.48f)),
	    darken(bodyGreen, 0.2f),
	    3.5f);
}

void Renderer2D::drawBrick(Vec2 pos, Vec2 size, Color fillColor, float rotationRad)
{
	// Get corners first
	Vec2 corners[4];
	rectCorners(pos, size, rotationRad, corners);
	
	// Main fill
	drawFilledQuad(corners, fillColor);
	
	// Highlight edge (top)
	Color highlightColor = lighten(fillColor, 0.12f);
	thickLine(corners[0], corners[3], highlightColor, 2.5f);
	
	// Shadow edge (bottom)
	Color shadowColor = darken(fillColor, 0.15f);
	thickLine(corners[1], corners[2], shadowColor, 2.5f);
	
	// Outline
	Color outlineColor = darken(fillColor, 0.25f);
	drawQuadOutline(corners, outlineColor, 2.f);
	
	// Horizontal stripes at fixed proportions
	Color stripeColor = darken(fillColor, 0.08f);
	stripeColor.a = 0.88f;
	
	Vec2 ctr(pos.x + size.x * 0.5f, pos.y + size.y * 0.5f);
	float proportions[] = {0.28f, 0.57f, 0.80f};
	for (float t : proportions)
	{
		Vec2 p0(pos.x, pos.y + size.y * t);
		Vec2 p1(pos.x + size.x, pos.y + size.y * t);
		if (rotationRad != 0.f)
		{
			p0 = rotatePoint(p0, ctr, rotationRad);
			p1 = rotatePoint(p1, ctr, rotationRad);
		}
		drawLine(p0, p1, stripeColor, 1.2f);
	}
}

void Renderer2D::drawSlingshot(Vec2 anchor, Vec2 pullPoint, float bandWidth)
{
	// Wood colors
	Color woodMain(0.45f, 0.32f, 0.22f, 1.f);
	Color woodLight(0.58f, 0.42f, 0.30f, 1.f);
	Color woodDark(0.35f, 0.25f, 0.18f, 1.f);
	Color bandColor(0.38f, 0.26f, 0.16f, 1.f);
	Color pouchColor(0.32f, 0.22f, 0.14f, 1.f);
	
	// Slingshot geometry
	float stemHeight = 82.f;
	float forkSpread = 18.f;
	float forkLength = 44.f;
	float baseWidth = 42.f;
	
	// Points
	Vec2 stemBottom = anchor + Vec2(0.f, stemHeight);
	Vec2 stemTop = anchor;
	Vec2 forkLStart = anchor + Vec2(-forkSpread, 0.f);
	Vec2 forkLEnd = anchor + Vec2(-forkSpread - 8.f, -forkLength);
	Vec2 forkRStart = anchor + Vec2(forkSpread, 0.f);
	Vec2 forkREnd = anchor + Vec2(forkSpread + 8.f, -forkLength);
	// Pull the attachment points slightly inward/downward so the rubber bands read shorter.
	Vec2 bandLAttach = forkLEnd + Vec2(4.f, 12.f);
	Vec2 bandRAttach = forkREnd + Vec2(-4.f, 12.f);
	
	// Draw wooden stem and forks
	// Stem
	thickLine(stemBottom, stemTop, woodMain, 8.f);
	thickLine(stemBottom + Vec2(1.5f, 0.f), stemTop + Vec2(1.5f, -2.f), woodLight, 3.f);
	
	// Left fork
	thickLine(forkLStart, forkLEnd, woodMain, 6.f);
	thickLine(forkLStart + Vec2(1.f, 1.f), forkLEnd + Vec2(1.f, 1.f), woodLight, 2.f);
	
	// Right fork
	thickLine(forkRStart, forkREnd, woodMain, 6.f);
	thickLine(forkRStart + Vec2(1.f, 1.f), forkREnd + Vec2(1.f, 1.f), woodLight, 2.f);
	
	// Base crossbar
	Vec2 baseL = stemBottom + Vec2(-baseWidth * 0.5f, -8.f);
	Vec2 baseR = stemBottom + Vec2(baseWidth * 0.5f, -8.f);
	thickLine(baseL, baseR, woodMain, 6.f);
	thickLine(baseL + Vec2(0.f, -1.f), baseR + Vec2(0.f, -1.f), woodDark, 3.f);
	
	// Draw rubber bands
	thickLine(bandLAttach, pullPoint, bandColor, bandWidth);
	thickLine(bandRAttach, pullPoint, bandColor, bandWidth);
	
	// Draw pouch
	drawFilledCircle(pullPoint, 7.f, pouchColor, 16);
}

void Renderer2D::drawTrajectory(Vec2 launchPos, Vec2 launchVel, float gravity, int dotCount,
                                float timeStep)
{
	if (dotCount < 1)
		return;
	Vec2 acc(0.f, gravity);
	for (int i = 0; i < dotCount; ++i)
	{
		float t = timeStep * static_cast<float>(i);
		Vec2 p = launchPos + launchVel * t + acc * (0.5f * t * t);
		float fade = 1.f - static_cast<float>(i) / static_cast<float>(dotCount);
		float fade2 = fade * fade;  // Square attenuation for smoother falloff
		
		// Dark trajectory dots stay readable against the bright scenic background.
		Color c(0.05f, 0.05f, 0.05f, 0.20f + 0.55f * fade2);
		
		// Larger and brighter at front, smaller and dimmer at back
		float r = 3.5f + 2.0f * fade2;
		drawFilledCircle(p, r, c, 16);
	}
}

void Renderer2D::spawnParticles(Vec2 origin, int count, Color baseColor)
{
	static std::mt19937 rng(std::random_device{}());
	std::uniform_real_distribution<float> ang(0.f, 2.f * 3.14159265358979323846f);
	std::uniform_real_distribution<float> speed(50.f, 200.f);
	std::uniform_real_distribution<float> lifeDist(0.25f, 0.75f);
	std::uniform_real_distribution<float> sizeDist(2.f, 6.f);
	std::uniform_real_distribution<float> jitter(-0.1f, 0.1f);

	int room = kMaxParticles - static_cast<int>(particles_.size());
	int toAdd = std::min(count, std::max(0, room));
	for (int i = 0; i < toAdd; ++i)
	{
		Particle p{};
		p.pos = origin;
		float a = ang(rng);
		p.vel = Vec2(std::cos(a), std::sin(a)) * speed(rng);
		p.maxLife = lifeDist(rng);
		p.life = p.maxLife;
		p.size = sizeDist(rng);
		p.color = Color(
			clamp01(baseColor.r + jitter(rng)),
			clamp01(baseColor.g + jitter(rng)),
			clamp01(baseColor.b + jitter(rng)),
			1.f);
		particles_.push_back(p);
	}
}

void Renderer2D::updateAndDrawParticles(float deltaTime)
{
	const Vec2 gravity(0.f, -140.f);
	for (Particle &p : particles_)
	{
		p.life -= deltaTime;
		p.vel += gravity * deltaTime;
		p.pos += p.vel * deltaTime;
	}

	particles_.erase(
		std::remove_if(particles_.begin(), particles_.end(),
		               [](const Particle &q) { return q.life <= 0.f; }),
		particles_.end());

	for (const Particle &p : particles_)
	{
		float a = p.maxLife > 1e-6f ? (p.life / p.maxLife) : 0.f;
		Color c = p.color;
		c.a *= a;
		drawFilledCircle(p.pos, p.size, c, 16);
	}
}

void Renderer2D::drawPausedOverlay(float screenW, float screenH)
{
	// Full-screen dimming layer.
	drawFilledRect(Vec2(0.f, 0.f), Vec2(screenW, screenH), Color(0.f, 0.f, 0.f, 0.22f));

	// Center panel behind the label.
	const Vec2 panelSize(340.f, 110.f);
	const Vec2 panelPos((screenW - panelSize.x) * 0.5f, screenH * 0.18f);
	drawFilledRect(panelPos, panelSize, Color(0.08f, 0.12f, 0.18f, 0.78f));
	drawRectOutline(panelPos, panelSize, Color(0.95f, 0.98f, 1.f, 0.85f), 3.f);

	const Color textColor(1.f, 0.98f, 0.84f, 0.95f);
	const float stroke = 8.f;
	const float letterW = 34.f;
	const float letterH = 48.f;
	const float gap = 14.f;
	const float innerGap = 12.f;
	const float totalW = letterW * 6.f + gap * 4.f + innerGap;
	const float startX = (screenW - totalW) * 0.5f;
	const float y = panelPos.y + 30.f;

	auto hline = [&](float x0, float x1, float yy)
	{
		drawLine(Vec2(x0, yy), Vec2(x1, yy), textColor, stroke);
	};
	auto vline = [&](float xx, float y0, float y1)
	{
		drawLine(Vec2(xx, y0), Vec2(xx, y1), textColor, stroke);
	};
	auto diag = [&](Vec2 a, Vec2 b)
	{
		drawLine(a, b, textColor, stroke);
	};

	float x = startX;

	// P
	vline(x, y, y + letterH);
	hline(x, x + letterW, y);
	hline(x, x + letterW, y + letterH * 0.5f);
	vline(x + letterW, y, y + letterH * 0.5f);

	x += letterW + gap;

	// A
	diag(Vec2(x, y + letterH), Vec2(x + letterW * 0.5f, y));
	diag(Vec2(x + letterW * 0.5f, y), Vec2(x + letterW, y + letterH));
	hline(x + letterW * 0.18f, x + letterW * 0.82f, y + letterH * 0.55f);

	x += letterW + gap;

	// U
	vline(x, y, y + letterH * 0.78f);
	vline(x + letterW, y, y + letterH * 0.78f);
	hline(x + letterW * 0.12f, x + letterW * 0.88f, y + letterH);

	x += letterW + gap;

	// S
	hline(x, x + letterW, y);
	hline(x, x + letterW, y + letterH * 0.5f);
	hline(x, x + letterW, y + letterH);
	vline(x, y, y + letterH * 0.5f);
	vline(x + letterW, y + letterH * 0.5f, y + letterH);

	x += letterW + innerGap;

	// E
	vline(x, y, y + letterH);
	hline(x, x + letterW, y);
	hline(x, x + letterW * 0.85f, y + letterH * 0.5f);
	hline(x, x + letterW, y + letterH);

	x += letterW + gap;

	// D
	vline(x, y, y + letterH);
	hline(x, x + letterW * 0.72f, y);
	hline(x, x + letterW * 0.72f, y + letterH);
	vline(x + letterW, y + letterH * 0.16f, y + letterH * 0.84f);
	drawLine(Vec2(x + letterW * 0.72f, y), Vec2(x + letterW, y + letterH * 0.16f), textColor, stroke);
	drawLine(Vec2(x + letterW * 0.72f, y + letterH), Vec2(x + letterW, y + letterH * 0.84f), textColor, stroke);
}

void Renderer2D::drawLine(Vec2 a, Vec2 b, Color color, float width)
{
	thickLine(a, b, color, width);
}

void Renderer2D::drawFilledRect(Vec2 pos, Vec2 size, Color color, float rotationRad)
{
	Vec2 corners[4];
	rectCorners(pos, size, rotationRad, corners);
	drawFilledQuad(corners, color);
}

void Renderer2D::drawRectOutline(Vec2 pos, Vec2 size, Color color, float lineWidth, float rotationRad)
{
	Vec2 corners[4];
	rectCorners(pos, size, rotationRad, corners);
	drawQuadOutline(corners, color, lineWidth);
}

void Renderer2D::drawFilledQuad(const Vec2 corners[4], Color color)
{
	const float v[] = {
		corners[0].x, corners[0].y, corners[1].x, corners[1].y, corners[2].x, corners[2].y,
		corners[0].x, corners[0].y, corners[2].x, corners[2].y, corners[3].x, corners[3].y};
	uploadAndDraw(v, 6, color);
}

void Renderer2D::drawQuadOutline(const Vec2 corners[4], Color color, float lineWidth)
{
	for (int i = 0; i < 4; ++i)
		thickLine(corners[i], corners[(i + 1) % 4], color, lineWidth);
}

void Renderer2D::drawCircleOutline(Vec2 center, float radius, Color color, float lineWidth, int segments)
{
	if (segments < 3) segments = 3;
	for (int i = 0; i < segments; ++i)
	{
		float a0 = (static_cast<float>(i)     / static_cast<float>(segments)) * kTwoPi;
		float a1 = (static_cast<float>(i + 1) / static_cast<float>(segments)) * kTwoPi;
		Vec2 p0 = center + Vec2(std::cos(a0), std::sin(a0)) * radius;
		Vec2 p1 = center + Vec2(std::cos(a1), std::sin(a1)) * radius;
		thickLine(p0, p1, color, lineWidth);
	}
}

void Renderer2D::drawFilledCircle(Vec2 center, float radius, Color color, int segments)
{
	if (segments < 3) segments = 3;
	const int maxVerts = segments * 3;
	static thread_local std::vector<float> circleVerts;
	circleVerts.resize(static_cast<size_t>(maxVerts * 2));
	float *buf = circleVerts.data();
	for (int i = 0; i < segments; ++i)
	{
		float a0 = (static_cast<float>(i)     / static_cast<float>(segments)) * kTwoPi;
		float a1 = (static_cast<float>(i + 1) / static_cast<float>(segments)) * kTwoPi;
		int base = i * 6;
		buf[base + 0] = center.x;
		buf[base + 1] = center.y;
		buf[base + 2] = center.x + std::cos(a0) * radius;
		buf[base + 3] = center.y + std::sin(a0) * radius;
		buf[base + 4] = center.x + std::cos(a1) * radius;
		buf[base + 5] = center.y + std::sin(a1) * radius;
	}
	uploadAndDraw(buf, maxVerts, color);
}

void Renderer2D::drawContactDebug(Vec2 point, Vec2 normal, bool visible)
{
	if (!visible)
		return;
	thickLine(point, point + normal * 100.f, Colors::Green, 4.f);
	drawFilledRect(Vec2(point.x - 3.f, point.y - 3.f), Vec2(6.f, 6.f), Colors::White);
}

void Renderer2D::flush()
{
}
