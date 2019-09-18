
// glm
#include <glm/gtc/random.hpp>

// project
#include "boid.hpp"
#include "scene.hpp"
#include "cgra/cgra_mesh.hpp"


using namespace glm;
using namespace std;


vec3 Boid::color() const {
    if(m_id == 1){
        return vec3(1, 1, 0);
    }
    else if(m_id == 2){
        return vec3(1, 0, 1);
    }
    else if(m_id == 3){
        return vec3(0, 1, 1);
    }
    else {
        return vec3(0, 1, 0);
    }
}


void Boid::calculateForces(Scene *scene) {
	//-------------------------------------------------------------
	// [Assignment 3] :
	// Calculate the forces affecting the boid and update the
	// acceleration (assuming mass = 1).
	// Do NOT update velocity or position in this function.
	// Core : 
	//  - Cohesion
	//  - Alignment
	//  - Avoidance
	//  - Soft Bound (optional)
	// Completion : 
	//  - Cohesion and Alignment with only boids in the same flock
	//  - Predator Avoidance (boids only)
	//  - Predator Chase (predator only)
	// Challenge : 
	//  - Obstacle avoidance
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...
    
    //used this page as a reference https://processing.org/examples/flocking.html
    
    m_acceleration += (cohesion(scene) + avoidance(scene) + alignment(scene))/1.0f;
    
}
glm::vec3 Boid::seek(glm::vec3 target, Scene *scene){
    glm::vec3 desired = scene->cohesionWeight*(target-m_position);
    glm::normalize(desired);
    desired = desired*scene->speed;
    glm::vec3 steer = scene->alignmentWeight*(desired - m_velocity);
    if(glm::length(steer) > maxForce){
        steer = maxForce * glm::normalize(steer);
    }
    return steer;
}
glm::vec3 Boid::cohesion(Scene *scene){
    sum = vec3(0, 0, 0);
    int count = 0;
    for(Boid b : scene->m_boids){
        for(Boid other : scene->m_boids){
            float d = glm::distance(b.m_position, other.m_position);
            if ((d > 0) && (d < scene->neighbourDist) && b.m_id == other.m_id) {
                sum += other.m_position; // Add position
                count++;
            }
        }
    }
        if(count>0){
            sum = sum/(float)count;
            //return sum;
            return seek(sum, scene);
        }
        else {
            return glm::vec3(0,0,0);
        }
}
glm::vec3 Boid::alignment(Scene *scene){
    sum = vec3(0, 0, 0);
    int count = 0;
    for(Boid other : scene->m_boids){
        float d = glm::distance(m_position, other.m_position);
        if ((d > 0) && (d < scene->neighbourDist) && m_id == other.m_id) {
            sum += other.m_velocity; // Add position
            count++;
        }
    }
    if(count>0){
        sum /= (float)count;
        glm::normalize(sum);
        sum *= scene->speed;
        glm::vec3 steer = scene->alignmentWeight*(sum - m_velocity);
        if(glm::length(steer) > maxForce){
            steer = maxForce * glm::normalize(steer);
        }
        return steer;
    }
    else {
        return glm::vec3(0,0,0);
    }
}
glm::vec3 Boid::avoidance(Scene *scene){
    glm::vec3 steer = vec3(0,0,0);
    int count = 0;
    for(Boid other : scene->m_boids){
        float d = glm::distance(m_position, other.m_position);
        if ((d > 0) && (d <= scene->neighbourDist) && m_id !=3) {
            glm::vec3 diff = m_position - other.m_position;
            float diffMag = glm::length(diff);
            diff /= diffMag*diffMag;
            
            steer += diff;
            count++;
        }
    }
    if(count>0){
        steer /= (float)count;
    }
    if(glm::length(steer)>0){
        glm::normalize(steer);
        steer *= scene->speed * scene->avoidanceWeight;
        steer -= m_velocity;
        if(glm::length(steer) > maxForce){
            steer = maxForce * glm::normalize(steer);
        }
//        for(Boid b : scene->m_boids){
//            if(b.m_id != m_id){
//                steer*=5;
//            }
//        }
    }
     return steer;
}

void Boid::update(float timestep, Scene *scene) {
	//-------------------------------------------------------------
	// [Assignment 3] :
	// Integrate the velocity of the boid using the timestep.
	// Update the position of the boid using the new velocity.
	// Take into account the bounds of the scene which may
	// require you to change the velocity (if bouncing) or
	// change the position (if wrapping).
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...

    if(m_position.x > scene->bound().x){
        m_position.x = -scene->bound().x;
    }
    else if(m_position.x < -scene->bound().x){
        m_position.x = scene->bound().x;
    }
    else if (m_position.y > scene->bound().y){
        m_position.y = -scene->bound().y;
    }
    else if(m_position.y < -scene->bound().y){
        m_position.y = scene->bound().y;
    }
    else if (m_position.z > scene->bound().z){
        m_position.z = -scene->bound().z;
    }
    else if(m_position.z < -scene->bound().z){
        m_position.z = scene->bound().z;
    }
    
    if(m_id == 3){
        if(glm::length(m_acceleration) > scene->predatorSpeed){
            m_acceleration = scene->speed * glm::normalize(m_acceleration);
        }
        
        
        m_velocity += m_acceleration * timestep;
        if(glm::length(m_velocity) > scene->predatorSpeed){
            m_velocity = scene->speed * glm::normalize(m_velocity);
        }
        
        m_position += m_velocity * timestep * scene->predatorSpeed;
        m_acceleration *= 0;
    }
    else{
        if(glm::length(m_acceleration) > scene->speed){
            m_acceleration = scene->speed * glm::normalize(m_acceleration);
        }
        
        
        m_velocity += m_acceleration * timestep;
        if(glm::length(m_velocity) > scene->speed){
            m_velocity = scene->speed * glm::normalize(m_velocity);
        }
        
        m_position += m_velocity * timestep * scene->speed;
        m_acceleration *= 0;
    }
}
