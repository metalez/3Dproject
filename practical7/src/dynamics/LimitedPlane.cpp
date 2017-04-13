#include "../../include/dynamics/LimitedPlane.hpp"
#include <glm/gtc/matrix_access.hpp>


LimitedPlane::LimitedPlane(const glm::vec3& normal, const glm::vec3& point, const glm::vec3 tr, const glm::vec3 bl)
    : m_n{normal}, m_d{dot(normal, point)}, m_tr{tr}, m_bl{bl}
{
}


LimitedPlane::~LimitedPlane()
{
}

glm::vec3 LimitedPlane::projectOnPlane(const glm::vec3& p)
{
    glm::vec3 planePoint = m_d*m_n;
    glm::vec3 v = p-planePoint;
    float dist = glm::dot(v,m_n);
    glm::vec3 projection = p - dist*m_n;
    return projection;
}

void LimitedPlane::setDistanceToOrigin(const float& d)
{
    m_d = d;
}

const float& LimitedPlane::distanceToOrigin() const
{
    return m_d;
}

void LimitedPlane::setNormal(const glm::vec3& n)
{
    m_n = n;
}

const glm::vec3& LimitedPlane::normal() const
{
    return m_n;
}
