#include "./../../include/dynamics/ParticleLimitedPlaneCollision.hpp"

ParticleLimitedPlaneCollision::ParticleLimitedPlaneCollision(ParticlePtr particle, LimitedPlanePtr plane, float restitution) :
    Collision(restitution)
{
    m_particle = particle;
    m_plane = plane;
}

ParticleLimitedPlaneCollision::~ParticleLimitedPlaneCollision()
{}

void ParticleLimitedPlaneCollision::do_solveCollision()
{

    //Don't process fixed particles (ground plane is assumed fixed)
    if (m_particle->isFixed())
        return;

    if (!m_plane->isGround)
    {
      glm::vec3 q = m_particle->getPosition();
      glm::vec3 proj = m_plane->projectOnPlane(q);
      float particlePlaneDist = glm::distance(q, proj);
      float penetration =  m_particle->getRadius()  - particlePlaneDist/2;
      glm::vec3 k = glm::normalize(m_plane->normal());

      m_particle->setPosition(q - (penetration*k));


      //Compute post-collision velocity
      glm::vec3 prev_v = m_particle->getVelocity();
      float proj_v = (1.0f + m_restitution)
          * glm::dot(k, prev_v)
          / (1.0 / m_particle->getMass());
      glm::vec3 new_v = prev_v - proj_v/m_particle->getMass()*k;
      m_particle->setVelocity(new_v);  

    }else{
      glm::vec3 q = m_particle->getPosition();
      glm::vec3 proj = m_plane->projectOnPlane(q);
      float particlePlaneDist = glm::distance(q, proj);
      float penetration =  m_particle->getRadius()  - particlePlaneDist;
      glm::vec3 k = glm::normalize(m_plane->normal());
      glm::vec3 newpos = q - penetration*k;

      newpos.z=m_plane->m_tr.z+1;
      glm::vec3 prev_v = m_particle->getVelocity();
      prev_v.z=0;
      m_particle->setVelocity(prev_v); 
      m_particle->setPosition(newpos);
    }








    //TODO: Solve ParticlePlane collisions,
    // update particle position and velocity after collision
    //
    //Functions to use:
    //glm::dot(v1, v2): Return the dot product of two vector.
    //Plane::distanceToOrigin(): Return the distance to origin from the plane
    //Plane::normal(): Return the normal of the plane
    //Particle::getRadius(), Particle::getPosition(), Particle::getVelocity(), Particle::setPosition(), Particle::setVelocity()


}



bool testParticleLimitedPlane(const ParticlePtr &particle, const LimitedPlanePtr &plane)
{
    /* Equation of a plane passing through A and normal to n:
   * dot( p - A, n ) = dot( p, n ) - dot( A, n ) = 0
   * dot( A, n ) is stored in the "distanceToOrigin" of the plane.
   *
   * Equation of a particle of radius r centered in c:
   * dot( p - c, p - c ) = r²
   *
   * distance( plane, particle )
   *   = min( distance( plane, c ) - r, 0 )              //definition
   *   = min( abs( dot( c - A, n ) ) - r, 0 )
   *   = min( abs( dot( c, n ) - dot( A, n ) ) - r, 0 )
   *
   * So, there is intersection if distance( plane, particle ) = 0
   * <=> abs( dot( c, n ) - dot( A, n ) ) - r <= 0
   * <=> abs( dot( c, n ) - dot( A, n ) ) <= r
   */

    //TODO: Test collision between particle and plane
    //Functions to use:
    //glm::dot(v1, v2): Return the dot product of two vector.
    //Plane::distanceToOrigin(): Return the distance to origin from the plane
    //Plane::normal(): Return the normal of the plane
    //Particle::getRadius(), Particle::getPosition()

    float distance = plane->distanceToOrigin();
    glm::vec3 c = particle->getPosition();
    glm::vec3 n = plane->normal();
    float distancePartPlane = abs(dot(c,n)-distance);
    float radius = particle->getRadius();


    if (distancePartPlane  <= radius ){
      if (c.x > plane->m_tr.x && c.x<plane->m_bl.x  && c.y> plane->m_tr.y  && c.y< plane->m_bl.y){
        return true;
      }else if (c.x < plane->m_tr.x && c.x>plane->m_bl.x  && c.y< plane->m_tr.y  && c.y> plane->m_bl.y){
        return true;
      }else if (c.y < plane->m_tr.y && c.y>plane->m_bl.y  && c.z< plane->m_tr.z  && c.z> plane->m_bl.z){
        return true;
      }else if (c.z < plane->m_tr.z && c.z>plane->m_bl.z  && c.y< plane->m_tr.y  && c.y> plane->m_bl.y){
        return true;
      }else if (c.x < plane->m_tr.x && c.x>plane->m_bl.x  && c.z< plane->m_tr.z  && c.z> plane->m_bl.z){
        return true;
      }else if (c.z < plane->m_tr.z && c.z>plane->m_bl.z  && c.x< plane->m_tr.x  && c.x> plane->m_bl.x){
        return true;
      }
        
    }


    return false;
}
