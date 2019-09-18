
#pragma once

//std
#include <vector>

// glm
#include <glm/glm.hpp>

// project
#include "cgra/cgra_mesh.hpp"
#include "cgra/cgra_shader.hpp"


// boid class (forward declaration)
class Boid;

class Scene {
private:
	// opengl draw data
	GLuint m_color_shader = 0;
	GLuint m_aabb_shader = 0;
	GLuint m_skymap_shader = 0;
	cgra::gl_mesh m_simple_boid_mesh;
	cgra::gl_mesh m_boid_mesh;
	cgra::gl_mesh m_predator_mesh;
	cgra::gl_mesh m_sphere_mesh;

	// draw status
	bool m_show_aabb = true;
	bool m_show_axis = false;
	bool m_show_skymap = false;

	// scene data
	glm::vec3 m_bound_hsize = glm::vec3(20);
	


	//-------------------------------------------------------------
	// [Assignment 3] :
	// Create variables for keeping track of the boid parameters
	// such as min and max speed etc. These paramters can either be
	// public, or private with getter functions.
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...

public:
    std::vector<Boid> m_boids;
    std::vector<Boid> m_obstacles;
	Scene();

	// functions that load the scene
	void loadCore();
	void loadCompletion();
	void loadChallenge();

	// called every frame, with timestep in seconds
	void update(float timestep);

	// called every frame, with the given projection and view matrix
	void draw(const glm::mat4 &proj, const glm::mat4 &view);

	// called every frame (to fill out a ImGui::TreeNode)
	void renderGUI();

	// returns a const reference to the boids vector
	const std::vector<Boid> & boids() const { return m_boids; }

	// returns the half-size of the bounding box (centered around the origin)
	glm::vec3 bound() const { return m_bound_hsize; }

	// YOUR CODE GOES HERE
	// ...
    float minSpeed = 1.0f;
    float maxSpeed = 15.0f;
    float speed = 5.0f;
    float neighbourDist = 20.f;
    glm::vec3 cohesion;
    glm::vec3 alignment;
    glm::vec3 avoidance;
    float alignmentWeight = 0.5f;
    float cohesionWeight = 0.5f;
    float avoidanceWeight = 5.0f;
    float predatorSpeed = 5.0f;
    float antiPredatorWeight = 5.0f;
    float predatorNeighbourDist = 20.f;
    float predatorChaseWeight = 3.0f;
    float predatorAvoidanceWeight = 5.0f;
    bool completion = false;
    bool challenge = false;
};
