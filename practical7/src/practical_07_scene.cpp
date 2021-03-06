#include "../include/Viewer.hpp"
#include "../include/FrameRenderable.hpp"

#include "../include/dynamics/DynamicSystem.hpp"
#include "../include/dynamics/DampingForceField.hpp"
#include "../include/dynamics/ConstantForceField.hpp"
#include "../include/dynamics/SpringForceField.hpp"
#include "../include/dynamics/EulerExplicitSolver.hpp"
#include "../include/dynamics/LimitedPlane.hpp"
#include "../include/dynamics_rendering/DynamicSystemRenderable.hpp"
#include "../include/dynamics_rendering/ParticleRenderable.hpp"
#include "../include/dynamics_rendering/ParticleListRenderable.hpp"
#include "../include/dynamics_rendering/ConstantForceFieldRenderable.hpp"
#include "../include/dynamics_rendering/SpringForceFieldRenderable.hpp"
#include "../include/dynamics_rendering/SpringListRenderable.hpp"
#include "../include/dynamics_rendering/ControlledForceFieldRenderable.hpp"
#include "../include/dynamics_rendering/QuadRenderable.hpp"
#include "../include/ShaderProgram.hpp"
#include "../include/Viewer.hpp"
#include "../include/FrameRenderable.hpp"
#include "../include/lighting/DirectionalLightRenderable.hpp"

#include "../include/texturing/TexturedPlaneRenderable.hpp"
#include "../include/texturing/TexturedCubeRenderable.hpp"
#include "../include/texturing/MultiTexturedCubeRenderable.hpp"
#include "../include/texturing/MipMapCubeRenderable.hpp"
#include "../include/texturing/TexturedMeshRenderable.hpp"

#include "../teachers/CircleRenderable.hpp"
#include "../include/FrameRenderable.hpp"
#include "../teachers/CylinderRenderable.hpp"
#include "../teachers/MeshRenderable.hpp"
#include "../include/dynamics_rendering/ParticleRenderable.hpp"
#include <glm/gtc/type_ptr.hpp>


#include "../teachers/Geometries.hpp"

void practical07_playPool(Viewer& viewer,
    DynamicSystemPtr& system, DynamicSystemRenderablePtr& systemRenderable);


void initialize_practical_07_scene(Viewer& viewer, unsigned int scene_to_load)
{
    //Set up a shader and add a 3D frame.
    ShaderProgramPtr flatShader =
        std::make_shared<ShaderProgram>("../shaders/flatVertex.glsl",
                                        "../shaders/flatFragment.glsl");
    viewer.addShaderProgram(flatShader);
    FrameRenderablePtr frame = std::make_shared<FrameRenderable>(flatShader);
    viewer.addRenderable(frame);

    //Initialize a dynamic system (Solver, Time step, Restitution coefficient)
    DynamicSystemPtr system = std::make_shared<DynamicSystem>();
    EulerExplicitSolverPtr solver = std::make_shared<EulerExplicitSolver>();
    system->setSolver(solver);
    system->setDt(0.001);

    //Create a renderable associated to the dynamic system
    //This renderable is responsible for calling DynamicSystem::computeSimulationStep()in the animate() function
    //It also handles some of the key/mouse events
    DynamicSystemRenderablePtr systemRenderable = std::make_shared<DynamicSystemRenderable>(system);
    
    practical07_playPool(viewer, system, systemRenderable);
    viewer.addRenderable(systemRenderable);
    //Finally, run the animation
    viewer.startAnimation();
}



void practical07_playPool(Viewer& viewer, DynamicSystemPtr& system, DynamicSystemRenderablePtr& systemRenderable)
{
    //Initialize a shader for the following renderables
    ShaderProgramPtr flatShader
        = std::make_shared<ShaderProgram>("../shaders/flatVertex.glsl","../shaders/flatFragment.glsl");
    viewer.addShaderProgram(flatShader);

    glm::mat4 parentTransformation(1.0), localTransformation(1.0);
    std::string filename;
    MaterialPtr pearl = Material::Pearl();
    MaterialPtr custom = Material::Custom();
    MaterialPtr customClear = Material::CustomClear();
    ShaderProgramPtr texShader
            = std::make_shared<ShaderProgram>("../shaders/textureVertex.glsl",
                                              "../shaders/textureFragment.glsl");
    viewer.addShaderProgram(texShader);

    //Position the camera
    viewer.getCamera().setViewMatrix(
        glm::lookAt(glm::vec3(0, -15, 15), glm::vec3(0,0,0), glm::vec3(0,0,1)) );

    //Define a directional light for the whole scene
    glm::vec3 d_direction = glm::normalize(glm::vec3(0.0,0.0,-1.0));
    glm::vec3 d_ambient(1.0,1.0,1.0), d_diffuse(1.0,1.0,0.8), d_specular(1.0,1.0,1.0);
    DirectionalLightPtr directionalLight = std::make_shared<DirectionalLight>(d_direction, d_ambient, d_diffuse, d_specular);
    //Add a renderable to display the light and control it via mouse/key event
    glm::vec3 lightPosition(0.0,0.0,5.0);
    DirectionalLightRenderablePtr directionalLightRenderable = std::make_shared<DirectionalLightRenderable>(flatShader, directionalLight, lightPosition);
    localTransformation = glm::scale(glm::mat4(1.0), glm::vec3(0.5,0.5,0.5));
    directionalLightRenderable->setLocalTransform(localTransformation);
    viewer.setDirectionalLight(directionalLight);
    //viewer.addRenderable(directionalLightRenderable);





    //Initialize two particles with position, velocity, mass and radius and add it to the system
    glm::vec3 px(0.0, 0.0, 0.0);
    glm::vec3 pv(0.0, 0.0, 0.0);
    float pm = 1.0, pr = 1.0;
    px = glm::vec3(0.0,0.0,3.0);
    ParticlePtr mobile = std::make_shared<Particle>( px, pv, pm, pr);
    system->addParticle( mobile );
    //link a mesh to a particle


    // px = glm::vec3(0.0,5.0,1.0);
    // ParticlePtr other = std::make_shared<Particle>( px, pv, pm, pr);
    // system->addParticle( other );

    //Create a particleRenderable for each particle of the system
    //Add them to the system renderable
    ParticleRenderablePtr mobileRenderable = std::make_shared<ParticleRenderable>(flatShader, mobile);
    //HierarchicalRenderable::addChild(systemRenderable, mobileRenderable);

    // textured bunny
    TexturedMeshRenderablePtr bunny =
        std::make_shared<TexturedMeshRenderable>(
            texShader, "../meshes/bunny.obj", "../textures/texturedBunny.png");
    bunny->setMaterial(custom);
    glm::vec3 particlePos= mobileRenderable->m_particle->getPosition();
    glm::mat4 transfo(1.0);
    //transfo = glm::rotate( transfo, float(M_PI_2), glm::vec3(0,1,0));
    transfo =glm::rotate(transfo,float(M_PI_2), glm::vec3(0,1,0));
    //parentTransformation = glm::scale( parentTransformation, glm::vec3(2,2,2));
   // transfo=glm::rotate( transfo, float(M_PI_2), glm::vec3(0,0,1));
    //transfo=glm::rotate( transfo, float(M_PI), glm::vec3(0,0,1));
    
    //transfo = glm::rotate( transfo, float(M_PI), glm::vec3(0,1,0));
    bunny->setParentTransform( glm::scale(transfo, glm::vec3(2,2,2)) );

    bunny->anchor=mobile;
    bunny->system=system;
    bunny->systemRenderable=systemRenderable;
    bunny->shader=flatShader;
    bunny->basePos=glm::vec3(0,0,0);
    viewer.addRenderable(bunny);





    //HierarchicalRenderable::addChild( mobile,bunny);    


    // ParticleRenderablePtr otherRenderable = std::make_shared<ParticleRenderable>(flatShader, other);
    // HierarchicalRenderable::addChild(systemRenderable, otherRenderable);

    //Initialize four planes to create walls arround the particles

    glm::vec3 planeNormal, planePoint;
    planeNormal = glm::vec3(-1, 0, 0);
    planePoint = glm::vec3(50, 0, 0);
    PlanePtr p0 = std::make_shared<Plane>(planeNormal, planePoint);
    system->addPlaneObstacle(p0);

    planeNormal = glm::vec3(1, 0, 0);
    planePoint = glm::vec3(-50, 0, 0);
    PlanePtr p1 = std::make_shared<Plane>(planeNormal, planePoint);
    system->addPlaneObstacle(p1);

    planeNormal = glm::vec3(0, -1, 0);
    planePoint = glm::vec3(0, 50, 0);
    PlanePtr p2 = std::make_shared<Plane>(planeNormal, planePoint);
    system->addPlaneObstacle(p2);

    planeNormal = glm::vec3(0, 1, 0);
    planePoint = glm::vec3(0, -50, 0);
    PlanePtr p3 = std::make_shared<Plane>(planeNormal, planePoint);
    system->addPlaneObstacle(p3);

    planeNormal = glm::vec3(0, 0, 1);
    planePoint = glm::vec3(0, 0, 0);
    PlanePtr floor = std::make_shared<Plane>( planeNormal, planePoint);
    floor->isGround=true;
    system->addPlaneObstacle(floor);


    planeNormal = glm::vec3(0, 0, 1);
    planePoint = glm::vec3(0, 0, 1);
    LimitedPlanePtr floor2 = std::make_shared<LimitedPlane>( planeNormal, planePoint, glm::vec3 (20,20,1), glm::vec3 (-20,-20,1));
    floor2->isGround=true;
    system->addLimitedPlaneObstacle(floor2);


    
    planeNormal = glm::vec3(-1, 0, 0);
    planePoint = glm::vec3(20, 0, 0);
    LimitedPlanePtr p4 = std::make_shared<LimitedPlane>(planeNormal, planePoint,glm::vec3(20,20,1),glm::vec3(20,-20,-1) );
    system->addLimitedPlaneObstacle(p4);

    planeNormal = glm::vec3(1, 0, 0);
    planePoint = glm::vec3(-20, 0, 0);
    LimitedPlanePtr p5 = std::make_shared<LimitedPlane>(planeNormal, planePoint,glm::vec3(-20,20,1),glm::vec3(-20,-20,-1));
    system->addLimitedPlaneObstacle(p5);

    planeNormal = glm::vec3(0, -1, 0);
    planePoint = glm::vec3(0, 20, 0);
    LimitedPlanePtr p6 = std::make_shared<LimitedPlane>(planeNormal, planePoint,glm::vec3(20,20,1),glm::vec3(-20,20,-1));
    system->addLimitedPlaneObstacle(p6);

    planeNormal = glm::vec3(0, 1, 0);
    planePoint = glm::vec3(0, -20, 0);
    LimitedPlanePtr p7 = std::make_shared<LimitedPlane>(planeNormal, planePoint,glm::vec3(20,-20,1),glm::vec3(-20,-20,-1));
    system->addLimitedPlaneObstacle(p7);

    //Textured plane
    filename = "../textures/fadeaway_dn2.tga";
    TexturedPlaneRenderablePtr texPlane_floor2 = std::make_shared<TexturedPlaneRenderable>(texShader, filename);

    // parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI_2), glm::vec3(1,0,0));
    // parentTransformation = glm::rotate( parentTransformation, -float(M_PI_2), glm::vec3(0,1,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(40,40,1));
    parentTransformation = glm::translate( parentTransformation, glm::vec3(0, 0, 1));

    texPlane_floor2->setParentTransform(parentTransformation);
    texPlane_floor2->setMaterial(customClear);
    texPlane_floor2->basePos=parentTransformation;
    HierarchicalRenderable::addChild(systemRenderable, texPlane_floor2);




    //Textured plane
    filename = "../textures/fadeaway_bk.tga";
    TexturedPlaneRenderablePtr texPlane_bk = std::make_shared<TexturedPlaneRenderable>(texShader, filename);

    parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI_2), glm::vec3(1,0,0));
    parentTransformation = glm::rotate( parentTransformation, -float(M_PI_2), glm::vec3(0,1,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(100,30,1));
    parentTransformation = glm::translate( parentTransformation, glm::vec3(0, 0, -49));

    texPlane_bk->setParentTransform(parentTransformation);
    texPlane_bk->setMaterial(custom);
    texPlane_bk->basePos=parentTransformation;
    HierarchicalRenderable::addChild(systemRenderable, texPlane_bk);

        //Textured plane
    filename = "../textures/fadeaway_dn2.tga";
    TexturedPlaneRenderablePtr texPlane_bk2 = std::make_shared<TexturedPlaneRenderable>(texShader, filename);

    parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI_2), glm::vec3(1,0,0));
    parentTransformation = glm::rotate( parentTransformation, -float(M_PI_2), glm::vec3(0,1,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(40,2,1));
    parentTransformation = glm::translate( parentTransformation, glm::vec3(0, 0, -20));

    texPlane_bk2->setParentTransform(parentTransformation);
    texPlane_bk2->setMaterial(custom);
    texPlane_bk2->basePos=parentTransformation;
    HierarchicalRenderable::addChild(systemRenderable, texPlane_bk2);

    filename = "../textures/fadeaway_lf.tga";
    TexturedPlaneRenderablePtr texPlane_lf = std::make_shared<TexturedPlaneRenderable>(texShader, filename);

    parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI_2), glm::vec3(1,0,0));
    //parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(0,1,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(100,30,1));
    parentTransformation = glm::translate( parentTransformation, glm::vec3(0, 0.003, -49));

    texPlane_lf->setParentTransform(parentTransformation);
    texPlane_lf->setMaterial(custom);
    texPlane_lf->basePos=parentTransformation;
    HierarchicalRenderable::addChild(systemRenderable, texPlane_lf);

    filename = "../textures/fadeaway_dn2.tga";
    TexturedPlaneRenderablePtr texPlane_lf2 = std::make_shared<TexturedPlaneRenderable>(texShader, filename);

    parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI_2), glm::vec3(1,0,0));
    //parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(0,1,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(40,2,1));
    parentTransformation = glm::translate( parentTransformation, glm::vec3(0, 0, -20));

    texPlane_lf2->setParentTransform(parentTransformation);
    texPlane_lf2->setMaterial(custom);
    texPlane_lf2->basePos=parentTransformation;
    HierarchicalRenderable::addChild(systemRenderable, texPlane_lf2);
     filename = "../textures/fadeaway_ft.tga";
    TexturedPlaneRenderablePtr texPlane_ft = std::make_shared<TexturedPlaneRenderable>(texShader, filename);

    parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI_2), glm::vec3(1,0,0));
    parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(0,1,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(100,30,1));
    parentTransformation = glm::translate( parentTransformation, glm::vec3(0, 0, -49));

    texPlane_ft->setParentTransform(parentTransformation);
    texPlane_ft->setMaterial(custom);
    texPlane_ft->basePos=parentTransformation;
    HierarchicalRenderable::addChild(systemRenderable, texPlane_ft);

    filename = "../textures/fadeaway_dn2.tga";
    TexturedPlaneRenderablePtr texPlane_ft2 = std::make_shared<TexturedPlaneRenderable>(texShader, filename);

    parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI_2), glm::vec3(1,0,0));
    parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(0,1,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(40,2,1));
    parentTransformation = glm::translate( parentTransformation, glm::vec3(0, 0, -20));

    texPlane_ft2->setParentTransform(parentTransformation);
    texPlane_ft2->setMaterial(custom);
    texPlane_ft2->basePos=parentTransformation;
    HierarchicalRenderable::addChild(systemRenderable, texPlane_ft2);



    filename = "../textures/fadeaway_rt.tga";
    TexturedPlaneRenderablePtr texPlane_rt = std::make_shared<TexturedPlaneRenderable>(texShader, filename);

    parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI_2), glm::vec3(1,0,0));
    //parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(0,1,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(100,30,1));
    parentTransformation = glm::translate( parentTransformation, glm::vec3(0, 0, 49));
    parentTransformation = glm::rotate( parentTransformation, float(M_PI), glm::vec3(0,1,0));
    texPlane_rt->setParentTransform(parentTransformation);
    texPlane_rt->setMaterial(custom);
    texPlane_rt->basePos=parentTransformation;
    HierarchicalRenderable::addChild(systemRenderable, texPlane_rt);

    filename = "../textures/fadeaway_dn2.tga";
    TexturedPlaneRenderablePtr texPlane_rt2 = std::make_shared<TexturedPlaneRenderable>(texShader, filename);

    parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI_2), glm::vec3(1,0,0));
    //parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(0,1,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(40,2,1));
    parentTransformation = glm::translate( parentTransformation, glm::vec3(0, 0, 20));
    parentTransformation = glm::rotate( parentTransformation, float(M_PI), glm::vec3(0,1,0));
    texPlane_rt2->setParentTransform(parentTransformation);
    texPlane_rt2->setMaterial(custom);
    texPlane_rt2->basePos=parentTransformation;
    HierarchicalRenderable::addChild(systemRenderable, texPlane_rt2);

    filename = "../textures/fadeaway_up.tga";
    TexturedPlaneRenderablePtr texPlane_up = std::make_shared<TexturedPlaneRenderable>(texShader, filename);

    parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI_2), glm::vec3(0,0,1));
    parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI), glm::vec3(0,1,0));

    parentTransformation = glm::scale( parentTransformation, glm::vec3(100,100,1));
    //parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(1,0,0));
    parentTransformation = glm::translate( parentTransformation, glm::vec3(0, 0, -15));
    //parentTransformation = glm::rotate( parentTransformation, float(M_PI), glm::vec3(0,1,0));
    texPlane_up->setParentTransform(parentTransformation);
    texPlane_up->setMaterial(custom);
    texPlane_up->basePos=parentTransformation;
    HierarchicalRenderable::addChild(systemRenderable, texPlane_up);


    
    filename = "../textures/fadeaway_dn.tga";
    TexturedPlaneRenderablePtr texPlane_dn = std::make_shared<TexturedPlaneRenderable>(texShader, filename);


	parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI_2), glm::vec3(0,0,1));
	parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI), glm::vec3(0,1,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(200,200,1));
    //parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(1,0,0));
    parentTransformation = glm::translate( parentTransformation, glm::vec3(0, 0, 1.5));
    //parentTransformation = glm::rotate( parentTransformation, float(M_PI), glm::vec3(0,1,0));
    texPlane_dn->setParentTransform(parentTransformation);
    texPlane_dn->setMaterial(customClear);
    texPlane_dn->basePos=parentTransformation;
    HierarchicalRenderable::addChild(systemRenderable, texPlane_dn);

/*
	TexturedPlaneRenderablePtr ground[5][5];
    for (int i=0;i<5;i++){
		    for (int j=0;j<5;j++){
				ground[i][j] = std::make_shared<TexturedPlaneRenderable>(texShader, filename);
    			parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI_2), glm::vec3(0,0,1));
    			parentTransformation = glm::rotate( glm::mat4(1.0), float(M_PI), glm::vec3(0,1,0));
				parentTransformation = glm::scale( parentTransformation, glm::vec3(20,20,1));
				parentTransformation = glm::translate( parentTransformation, glm::vec3(i, j, 0));
				ground[i][j]->setParentTransform(parentTransformation);
   				ground[i][j]->setMaterial(custom);
    			ground[i][j]->basePos=parentTransformation;
    			HierarchicalRenderable::addChild(systemRenderable, ground[i][j]);
    	}
    }
*/

    //Initialize a force field that apply only to the mobile particle
    glm::vec3 nullForce(0.0, 0.0, 0.0);
    std::vector<ParticlePtr> vParticle;
    vParticle.push_back(mobile);
    ConstantForceFieldPtr force = std::make_shared<ConstantForceField>(vParticle, nullForce);
    system->addForceField(force);


    //Initialize a renderable for the force field applied on the mobile particle.
    //This renderable allows to modify the attribute of the force by key/mouse events
    //Add this renderable to the systemRenderable.
    ControlledForceFieldRenderablePtr forceRenderable = std::make_shared<ControlledForceFieldRenderable>(flatShader, force);
    forceRenderable->camera=&viewer.getCamera(); 
    forceRenderable->texPlane_bk =texPlane_bk;
    forceRenderable->texPlane_ft =texPlane_ft;
    forceRenderable->texPlane_lf =texPlane_lf;
    forceRenderable->texPlane_rt =texPlane_rt;
    forceRenderable->texPlane_up =texPlane_up;
    HierarchicalRenderable::addChild(systemRenderable, forceRenderable);
    bunny->field=forceRenderable;
    bunny->system=system;
    //Add a damping force field to the mobile.
    DampingForceFieldPtr dampingForceField = std::make_shared<DampingForceField>(vParticle, 15.9);
    system->addForceField(dampingForceField);

    //Activate collision and set the restitution coefficient to 1.0
    system->setCollisionsDetection(true);
    system->setRestitution(1.0f);
 
    // values for position base
    float bx  = 0.0 , by = 0.0 , bz = 0.0;
    px = glm::vec3(5.0, 5.0, 3);
    pv = glm::vec3(0.0, 0.0, 0.0);
    ParticlePtr particle = std::make_shared<Particle>( px, pv, 4, pr);

    // Temporary variables to use to define transformation
    glm::mat4 rotationM(1.0), rot1(1.0), rot2(1.0);
    glm::mat4 scaleM(1.0);
    glm::mat4 translationM(1.0);

    // create renderable objects
    viewer.addRenderable(std::make_shared<FrameRenderable>(flatShader));

    // First Base Shpere of Snowman
    std::shared_ptr<ParticleRenderable> Pbase
        = std::make_shared<ParticleRenderable>(flatShader, particle);
    system->addParticle(particle);
    Pbase->system=system;
    Pbase->shader=flatShader;
    Pbase->setAnchor(particle);
    Pbase->basePos=glm::vec3(0,0,0);
    Pbase->systemRenderable=systemRenderable;


    //Add a damping force field to the mobile.
    std::vector<ParticlePtr> vParticle2;
    vParticle2.push_back(particle);
    DampingForceFieldPtr dampingForceField2 = std::make_shared<DampingForceField>(vParticle2, 240.9);
    system->addForceField(dampingForceField2);



    // Second middle sphere of snowman
    ParticlePtr particle_mid = std::make_shared<Particle>( px+glm::vec3(0,0,0.75), pv, 0, 3*pr/4);
    std::shared_ptr<ParticleRenderable> Pmid
        = std::make_shared<ParticleRenderable>(flatShader, particle_mid);
    Pmid->setAnchor(particle);
    Pmid->basePos=glm::vec3(0,0,1.50);
    Pmid->scale=glm::vec3(0.75,0.75,0.75);
    // //Head sphere of the snowman
    // ParticlePtr particle_head = std::make_shared<Particle>( px+glm::vec3(0,0,1.35), pv, 0, pr/2);
    // std::shared_ptr<ParticleRenderable> Phead
    //     = std::make_shared<ParticleRenderable>(flatShader, particle_head);
    // //translationM = glm::translate(glm::mat4(1.0), glm::vec3(0,0,bz+pr*5/4-0.05 )); //0.05 shift value so the two speheres seem connected
    // //Phead -> setParentTransform(translationM); 
    // Phead->setAnchor(particle);



    // textured hat
    TexturedMeshRenderablePtr hat =
        std::make_shared<TexturedMeshRenderable>(
            texShader, "../meshes/HatEatingHat.obj", "../textures/Hateatinghat1Tex.png");
    hat->setMaterial(custom);
    //particlePos= particle->getPosition();
    //transfo =glm::translate(glm::mat4(1.0),particlePos+glm::vec3(0,0,2));

    //parentTransformation = glm::scale( parentTransformation, glm::vec3(2,2,2));
    //transfo=glm::rotate( transfo, float(M_PI_2), glm::vec3(0,0,1));
    //transfo=glm::rotate( transfo, float(M_PI), glm::vec3(0,0,1));
    //hat->setParentTransform(transfo);
    //bunny->system=system;
    //bunny->systemRenderable=systemRenderable;
    //hat->shader=texShader;
    hat->basePos=glm::vec3(1,0,1.75);
    hat->setAnchor(particle);
    viewer.addRenderable(hat);
    // //Create Snowman hat
    // std::shared_ptr<teachers::CylinderRenderable> hat
    //     = std::make_shared<teachers::CylinderRenderable>(flatShader);
    // translationM = glm::translate(glm::mat4(),glm::vec3(0.0,0.0,bz+0.7-0.05));
    // hat -> setParentTransform(translationM);
    // scaleM = glm::scale(glm::mat4(1.0), glm::vec3(0.5,0.5,1.0));
    // hat -> setLocalTransform(scaleM);

    // std::shared_ptr<teachers::CircleRenderable> nose
    //     = std::make_shared<teachers::CircleRenderable>(flatShader);
    // translationM = glm::translate(glm::mat4(),glm::vec3(0.0,0.4,bz+0.6-0.05));
    // rotationM= glm::rotate(glm::mat4(1.0), (float)(M_PI/2.0), glm::vec3(1,0,0));
    // nose -> setParentTransform(rotationM);
    // scaleM = glm::scale(glm::mat4(1.0), glm::vec3(0.1,0.1,0.5));
    // nose -> setLocalTransform(translationM*scaleM);


    //HierarchicalRenderable::addChild(Pbase, Pmid);
    // HierarchicalRenderable::addChild(Pmid, Phead);
    // HierarchicalRenderable::addChild(Phead,hat);
    // HierarchicalRenderable::addChild(Phead,nose);
    viewer.addRenderable(Pbase);
 viewer.addRenderable(Pmid);


    // textured tree
    TexturedMeshRenderablePtr tree =
        std::make_shared<TexturedMeshRenderable>(
            texShader, "../meshes/fir.obj", "../textures/branch.png");
    tree->setMaterial(pearl);
    parentTransformation = glm::translate( glm::mat4(1.0), glm::vec3(24, 4, 0.0));
    parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(1,0,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(2,2,2));
    tree->setParentTransform( parentTransformation );
    viewer.addRenderable(tree);

    // textured tree2
    TexturedMeshRenderablePtr tree2 =
        std::make_shared<TexturedMeshRenderable>(
            texShader, "../meshes/fir.obj", "../textures/branch.png");
    tree2->setMaterial(pearl);
    parentTransformation = glm::translate( glm::mat4(1.0), glm::vec3(20, 14, 0.0));
    parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(1,0,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(1.5,2,2));
    tree2->setParentTransform( parentTransformation );
    viewer.addRenderable(tree2);

    // textured tree2
    TexturedMeshRenderablePtr tree22 =
        std::make_shared<TexturedMeshRenderable>(
            texShader, "../meshes/fir.obj", "../textures/branch.png");
    tree22->setMaterial(pearl);
    parentTransformation = glm::translate( glm::mat4(1.0), glm::vec3(20, 8, 0.0));
    parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(1,0,0));
    parentTransformation = glm::scale( parentTransformation, glm::vec3(4,2,2));
    tree22->setParentTransform( parentTransformation );
    viewer.addRenderable(tree22);

    // textured tree3
     TexturedMeshRenderablePtr tree3 =
        std::make_shared<TexturedMeshRenderable>(
            texShader, "../meshes/fir.obj", "../textures/branch.png");
     tree3->setMaterial(pearl);
     parentTransformation = glm::translate( glm::mat4(1.0), glm::vec3(34, 3, 0.0));
     parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(1,0,0));
     parentTransformation = glm::scale( parentTransformation, glm::vec3(2,2,4));
     tree3->setParentTransform( parentTransformation );
     viewer.addRenderable(tree3);

     // textured tree4
     TexturedMeshRenderablePtr tree4 =
        std::make_shared<TexturedMeshRenderable>(
            texShader, "../meshes/fir.obj", "../textures/branch.png");
     tree4->setMaterial(pearl);
     parentTransformation = glm::translate( glm::mat4(1.0), glm::vec3(29, 3, 0.0));
     parentTransformation = glm::rotate( parentTransformation, float(M_PI_2), glm::vec3(1,0,0));
     parentTransformation = glm::scale( parentTransformation, glm::vec3(5,3,1));
     tree4->setParentTransform( parentTransformation );
     viewer.addRenderable(tree4);


    DynamicSystemPtr system2 = std::make_shared<DynamicSystem>();
    EulerExplicitSolverPtr solver2 = std::make_shared<EulerExplicitSolver>();
    system2->setSolver(solver2);
    system2->setDt(0.001);
    //Create a renderable associated to the dynamic system
    DynamicSystemRenderablePtr systemRenderable2 = std::make_shared<DynamicSystemRenderable>(system2);
    viewer.addRenderable(systemRenderable2);
    system2->setCollisionsDetection(true);
    system2->setRestitution(1.0f);
    // snow
    std::shared_ptr<Particle>* s = new std::shared_ptr<Particle>[100];
    std::shared_ptr<ParticleRenderable>* sr = new std::shared_ptr<ParticleRenderable>[100];
    for(int i = 0; i < 17; i++) {
	  for(int j = 0; j < 17; j++) {
	  	int nj = rand()%40-20;
		int ni = (rand()%40)-20;
		int size = rand()%15;
		px = glm::vec3(ni, nj, size);
		s[i] = std::make_shared<Particle>( px, pv, pm, pr/18);
		sr[i] = std::make_shared<ParticleRenderable>(flatShader, s[i]);
        sr[i]->isSnow=true;
        sr[i]->setAnchor(s[i]);
		system2->addParticle(s[i]);
		viewer.addRenderable(sr[i]);
	  }  
    }

    //Initialize a force field that apply to all the particles of the system to simulate gravity
    //Add it to the system as a force field
    ConstantForceFieldPtr gravityForceField = std::make_shared<ConstantForceField>(system->getParticles(), glm::vec3{0,0,-23} );
    system->addForceField(gravityForceField);
    bunny->gravity=gravityForceField;
    Pbase->gravity=gravityForceField;


    ConstantForceFieldPtr gravityForceField2 = std::make_shared<ConstantForceField>(system2->getParticles(), glm::vec3{0.0,0.0,-1.2} );
    system2->addForceField(gravityForceField2);

}
