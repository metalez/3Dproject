#include "./../../include/dynamics_rendering/ParticleRenderable.hpp"
#include "./../../include/gl_helper.hpp"
#include "./../../include/log.hpp"
#include "./../../include/Utils.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>

ParticleRenderable::~ParticleRenderable()
{
    glcheck(glDeleteBuffers(1, &m_pBuffer));
    glcheck(glDeleteBuffers(1, &m_cBuffer));
    glcheck(glDeleteBuffers(1, &m_nBuffer));
}

ParticleRenderable::ParticleRenderable(ShaderProgramPtr shaderProgram, ParticlePtr particle) :
    HierarchicalRenderable(shaderProgram),
    m_particle(particle),
    m_pBuffer(0), m_cBuffer(0), m_nBuffer(0)
{
    basePos=particle->getPosition();
    double radius = 1.0;
    int thetaStep = 20;
    int phiStep = 10;

    glm::vec3 center(0.0, 0.0, 0.0);

    for (int i = 0; i < thetaStep; ++i) {
        for(int j = 0; j < phiStep; ++j) {
            double curr_theta = i*(2.0*M_PI / (double) thetaStep);
            double curr_phi = j*(M_PI / (double)phiStep);

            double next_theta = (i+1) * (2.0 * M_PI / (double) thetaStep);
            double next_phi = (j+1) * (M_PI / (double) phiStep);

            glm::vec3 faceNormal(1, 0, 0);
            std::array<glm::vec3,3> vTriangles;
            vTriangles[0] = center
                + glm::vec3(radius * cos(curr_theta) * sin(curr_phi),
                            radius * sin(curr_theta) * sin(curr_phi),
                            radius * cos(curr_phi));
            vTriangles[1] = center
                + glm::vec3(radius * cos(next_theta) * sin(curr_phi),
                            radius * sin(next_theta) * sin(curr_phi),
                            radius * cos(curr_phi));
            vTriangles[2] = center
                + glm::vec3(radius * cos(next_theta) * sin(next_phi),
                            radius * sin(next_theta) * sin(next_phi),
                            radius * cos(next_phi));
            faceNormal = -glm::normalize(glm::cross(vTriangles[1]-vTriangles[0], vTriangles[2]-vTriangles[0]));

            m_positions.push_back(vTriangles[0]);
            m_positions.push_back(vTriangles[1]);
            m_positions.push_back(vTriangles[2]);
            m_normals.push_back(faceNormal);
            m_normals.push_back(faceNormal);
            m_normals.push_back(faceNormal);
            m_colors.push_back(glm::vec4(255,50,250,1.0));
            m_colors.push_back(glm::vec4(55,20,50,1.0));
            m_colors.push_back(glm::vec4(255,250,250,1.0));

            vTriangles[0] = center
                + glm::vec3(radius * cos(curr_theta) * sin(curr_phi),
                            radius * sin(curr_theta) * sin(curr_phi),
                            radius * cos(curr_phi));
            vTriangles[1] = center
                + glm::vec3(radius * cos(next_theta) * sin(next_phi),
                            radius * sin(next_theta) * sin(next_phi),
                            radius * cos(next_phi));
            vTriangles[2] = center
                + glm::vec3(radius * cos(curr_theta) * sin(next_phi),
                            radius * sin(curr_theta) * sin(next_phi),
                            radius * cos(next_phi));
            faceNormal = -glm::normalize(glm::cross( vTriangles[1]-vTriangles[0], vTriangles[2]-vTriangles[0] ));

            m_positions.push_back(vTriangles[0]);
            m_positions.push_back(vTriangles[1]);
            m_positions.push_back(vTriangles[2]);
            m_normals.push_back(faceNormal);
            m_normals.push_back(faceNormal);
            m_normals.push_back(faceNormal);
            m_colors.push_back(glm::vec4(25,20,20,1.0));
            m_colors.push_back(glm::vec4(255,50,250,1.0));
            m_colors.push_back(glm::vec4(255,250,20,1.0));
        }
    }

    //Create buffers
    glGenBuffers(1, &m_pBuffer); //vertices
    glGenBuffers(1, &m_cBuffer); //colors
    glGenBuffers(1, &m_nBuffer); //normals

    //Activate buffer and send data to the graphics card
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_pBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_positions.size()*sizeof(glm::vec3), m_positions.data(), GL_STATIC_DRAW));
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_cBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_colors.size()*sizeof(glm::vec4), m_colors.data(), GL_STATIC_DRAW));
    glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_nBuffer));
    glcheck(glBufferData(GL_ARRAY_BUFFER, m_normals.size()*sizeof(glm::vec3), m_normals.data(), GL_STATIC_DRAW));
}

void ParticleRenderable::do_draw()
{
    //Update the parent and local transform matrix to position the geometric data according to the particle's data.
    const float& pRadius = m_particle->getRadius();
    const glm::vec3& pPosition = m_particle->getPosition();
    glm::mat4 scale = glm::scale(glm::mat4(1.0), glm::vec3(pRadius));
    glm::mat4 translate = glm::translate(glm::mat4(1.0), glm::vec3(pPosition));
    setLocalTransform(translate*scale);

    //Draw geometric data
    int positionLocation = m_shaderProgram->getAttributeLocation("vPosition");
    int colorLocation = m_shaderProgram->getAttributeLocation("vColor");
    int normalLocation = m_shaderProgram->getAttributeLocation("vNormal");
    int modelLocation = m_shaderProgram->getUniformLocation("modelMat");

    if (modelLocation != ShaderProgram::null_location) {
        glcheck(glUniformMatrix4fv(modelLocation, 1, GL_FALSE, glm::value_ptr(getModelMatrix())));
    }

    if (positionLocation != ShaderProgram::null_location) {
        glcheck(glEnableVertexAttribArray(positionLocation));
        glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_pBuffer));
        glcheck(glVertexAttribPointer(positionLocation, 3, GL_FLOAT, GL_FALSE, 0, (void*)0));
    }

    if (colorLocation != ShaderProgram::null_location) {
        glcheck(glEnableVertexAttribArray(colorLocation));
        glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_cBuffer));
        glcheck(glVertexAttribPointer(colorLocation, 4, GL_FLOAT, GL_FALSE, 0, (void*)0));
    }

    if (normalLocation != ShaderProgram::null_location) {
        glcheck(glEnableVertexAttribArray(normalLocation));
        glcheck(glBindBuffer(GL_ARRAY_BUFFER, m_nBuffer));
        glcheck(glVertexAttribPointer(normalLocation, 3, GL_FLOAT, GL_FALSE, 0, (void*)0));
    }

    //Draw triangles elements
    glcheck(glDrawArrays(GL_TRIANGLES,0, m_positions.size()));

    if (positionLocation != ShaderProgram::null_location) {
        glcheck(glDisableVertexAttribArray(positionLocation));
    }
    if (colorLocation != ShaderProgram::null_location) {
        glcheck(glDisableVertexAttribArray(colorLocation));
    }
    if (normalLocation != ShaderProgram::null_location) {
        glcheck(glDisableVertexAttribArray(normalLocation));
    }
}

void ParticleRenderable::do_animate(float time)
{ 

    if (isSnow && anchor!=NULL){
        if ((anchor->getPosition()).z< 0){
            anchor->incrPosition(glm::vec3(0,0,15));
        }
        return;
    }
    glm::mat4 transfo;

    if (anchor!=NULL){
        transfo = glm::translate(glm::mat4(1.0), anchor->getPosition());
        transfo = glm::translate(transfo, basePos);
        transfo=glm::scale(transfo, scale);
        setLocalTransform(transfo);
    }


    if (shader!=NULL && system!=NULL && gravity!=NULL && anchor!=NULL && int(time)%3==0){
        //gravity->rmParticle(mobile);

        float pm = 1, pr = 0.1;
        ParticlePtr mobile = std::make_shared<Particle>( anchor->getPosition()+glm::vec3(1,0,-0.2), glm::vec3(1,0,0), pm, pr);
        system->addParticle( mobile );
        ParticleRenderablePtr mobileRenderable = std::make_shared<ParticleRenderable>(shader, mobile);
        HierarchicalRenderable::addChild(systemRenderable, mobileRenderable);
        gravity->addParticle(mobile);
        mobile = std::make_shared<Particle>( anchor->getPosition()+glm::vec3(-1,0,-0.2), glm::vec3(-1,0,0), pm, pr);
        system->addParticle( mobile );
        mobileRenderable = std::make_shared<ParticleRenderable>(shader, mobile);
        HierarchicalRenderable::addChild(systemRenderable, mobileRenderable);
        gravity->addParticle(mobile);
        mobile = std::make_shared<Particle>( anchor->getPosition()+glm::vec3(0,1,-0.2), glm::vec3(0,1,0), pm, pr);
        system->addParticle( mobile );
        mobileRenderable = std::make_shared<ParticleRenderable>(shader, mobile);
        HierarchicalRenderable::addChild(systemRenderable, mobileRenderable);
        gravity->addParticle(mobile);
        mobile = std::make_shared<Particle>( anchor->getPosition()+ glm::vec3(0,-1,-0.2), glm::vec3(0,-1,0), pm, pr);
        system->addParticle( mobile );
        mobileRenderable = std::make_shared<ParticleRenderable>(shader, mobile);
        HierarchicalRenderable::addChild(systemRenderable, mobileRenderable);
        gravity->addParticle(mobile);
    }


}
void ParticleRenderable::setAnchor(ParticlePtr particle){
    anchor=particle;
    
}