#include "scenefluid.h"
#include "glutils.h"
#include "model.h"
#include <QOpenGLFunctions_3_3_Core>

SceneFluid::SceneFluid()
{
    widget = new WidgetFluid();
    connect(widget, SIGNAL(updatedParameters()), this, SLOT(updateSimParams()));
}

SceneFluid::~SceneFluid()
{
    if (widget) delete widget;
    if (shader) delete shader;
    if (vaoSphere) delete vaoSphere;
    if (fGravity) delete fGravity;
}

void SceneFluid::initialize()
{
    // Load shader
    shader = glutils::loadShaderProgram(":/shaders/phong.vert", ":/shaders/phong.frag");

    // Create a sphere VAO for particles
    Model sphere = Model::createIcosphere(2);
    vaoSphere = glutils::createVAO(shader, &sphere, buffers);
    numFacesSphere = sphere.numFaces();

    // Forces
    fGravity = new ForceConstAcceleration();
    system.addForce(fGravity);

    // Create cube (bath tub)
    Model cube = Model::createOpenCube();
    vaoCube = glutils::createVAO(shader, &cube, buffers);
    glutils::checkGLError();

    // bottom (y = 0), normal up
    colliderBottom.setPlane(Vec3(0, 1, 0), 0.0);

    // left wall (x = -100), normal +X
    colliderLeft.setPlane(Vec3(1, 0, 0), 100.0);

    // right wall (x = +100), normal -X
    colliderRight.setPlane(Vec3(-1, 0, 0), -100.0);

    // front wall (z = -100), normal +Z
    colliderFront.setPlane(Vec3(0, 0, 1), 100.0);

    // back wall (z = +100), normal -Z
    colliderBack.setPlane(Vec3(0, 0, -1), -100.0);




    //colliderBox.setFromCenterSize(Vec3(0, 50, 0), Vec3(100, 50, 100));

    tapPosition = Vec3(0, 120, 0);  // position above the tub
    emitRate = 150;                 // particles per second
    maxParticleLife = 20.0;         // seconds
    reset();
}

void SceneFluid::reset()
{
    system.deleteParticles();
    fGravity->clearInfluencedParticles();

    // Create a cube of particles
    const double spacing = 8.0;
    for (double x = -60; x < 60; x += spacing)
        for (double y = 10; y < 90; y += spacing)
            for (double z = -60; z < 60; z += spacing)     {
                Particle* p = new Particle();
                p->pos = Vec3(x, y, z);
                p->prevPos = p->pos;
                p->vel = Vec3(0, 0, 0);
                const double spacing = 6.0;
                p->radius = 2.0;
                p->color = Vec3(0.2, 0.4, 1.0);  // light blue
                system.addParticle(p);
                fGravity->addInfluencedParticle(p);
            }

    system.addForce(fGravity);
    fGravity->setAcceleration(Vec3(0, -9.81, 0));
}

void SceneFluid::updateSimParams()
{
    // Get values from widget
    fGravity->setAcceleration(Vec3(0, -widget->getGravity(), 0));

    restDensity = widget->getRestDensity();
    stiffness   = widget->getStiffness();
    viscosity   = widget->getViscosity();
    smoothingRadius = widget->getSmoothingRadius();
    particleMass = widget->getParticleMass();
}

void SceneFluid::emitParticles(double dt)
{
    int emitCount = std::max(1, int(std::round(emitRate * dt)));

    for (int i = 0; i < emitCount; ++i) {
        Particle* p;
        if (!deadParticles.empty()) {
            p = deadParticles.front();
            deadParticles.pop_front();
        } else {
            p = new Particle();
            system.addParticle(p);
            fGravity->addInfluencedParticle(p);
        }

        // Initialize new particle
        p->color = Vec3(0.2, 0.4, 1.0);
        p->radius = 2.0;
        p->life = maxParticleLife;

        // small random offset for natural flow
        double rx = Random::get(-2.0, 2.0);
        double rz = Random::get(-2.0, 2.0);

        p->pos = tapPosition + Vec3(rx, 0, rz);
        p->vel = Vec3(0, -Random::get(40.0, 60.0), 0);

        // Give it a valid previous position based on its velocity
        p->prevPos = p->pos - p->vel * 0.016;

    }
}


void SceneFluid::update(double dt)
{
    // Emit new particles each frame
    emitParticles(dt);

    // SPH or basic gravity simulation
    computeDensityAndPressure();
    computeForces();
    integrator.step(system, dt);

    // Handle wall collisions
    Collision col;
    for (Particle* p : system.getParticles()) {
        if (colliderBottom.testCollision(p, col))
            colliderBottom.resolveCollision(p, col, 0.5, 0.3);
        if (colliderLeft.testCollision(p, col))
            colliderLeft.resolveCollision(p, col, 0.5, 0.3);
        if (colliderRight.testCollision(p, col))
            colliderRight.resolveCollision(p, col, 0.5, 0.3);
        if (colliderFront.testCollision(p, col))
            colliderFront.resolveCollision(p, col, 0.5, 0.3);
        if (colliderBack.testCollision(p, col))
            colliderBack.resolveCollision(p, col, 0.5, 0.3);
    }



    for (Particle* p : system.getParticles()) {
        if (p->life > 0) {
            p->life -= dt;
            if (p->life < 0)
                deadParticles.push_back(p);
        }
    }
}


void SceneFluid::paint(const Camera& cam)
{
    QOpenGLFunctions* glFuncs = nullptr;
    glFuncs = QOpenGLContext::currentContext()->functions();

    shader->bind();

    // camera matrices
    QMatrix4x4 camProj = cam.getPerspectiveMatrix();
    QMatrix4x4 camView = cam.getViewMatrix();
    shader->setUniformValue("ProjMatrix", camProj);
    shader->setUniformValue("ViewMatrix", camView);

    // lighting
    const int numLights = 1;
    const QVector3D lightPosWorld[numLights] = { QVector3D(80, 80, 80) };
    const QVector3D lightColor[numLights]    = { QVector3D(1, 1, 1) };
    QVector3D lightPosCam[numLights];
    for (int i = 0; i < numLights; i++) {
        lightPosCam[i] = camView.map(lightPosWorld[i]);
    }
    shader->setUniformValue("numLights", numLights);
    shader->setUniformValueArray("lightPos", lightPosCam, numLights);
    shader->setUniformValueArray("lightColor", lightColor, numLights);

    // draw semi-transparent bath tub
    vaoCube->bind();

    glFuncs->glEnable(GL_BLEND);
    glFuncs->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glFuncs->glDisable(GL_CULL_FACE);  // show inner walls too

    // tub walls material
    shader->setUniformValue("matdiff", 0.6f, 0.8f, 1.0f);  // lighter blue
    shader->setUniformValue("matspec", 0.2f, 0.2f, 0.2f);
    shader->setUniformValue("matshin", 30.0f);


    // transform the cube
    QMatrix4x4 modelMat;
    modelMat.translate(0, 50, 0);   // center of the box
    modelMat.scale(100, 50, 100);   // make it wide and low
    shader->setUniformValue("ModelMatrix", modelMat);

    // draw all faces
    glFuncs->glDrawElements(GL_TRIANGLES, 3 * 2 * 5, GL_UNSIGNED_INT, 0);

    glFuncs->glEnable(GL_CULL_FACE);
    glFuncs->glDisable(GL_BLEND);
    vaoCube->release();

    // draw particles
    vaoSphere->bind();
    for (Particle* p : system.getParticles()) {
        QMatrix4x4 modelMat;
        modelMat.translate(p->pos.x(), p->pos.y(), p->pos.z());
        modelMat.scale(p->radius);
        shader->setUniformValue("ModelMatrix", modelMat);

        shader->setUniformValue("matdiff", p->color[0], p->color[1], p->color[2]);
        shader->setUniformValue("matspec", 1.0f, 1.0f, 1.0f);
        shader->setUniformValue("matshin", 80.0f);

        glFuncs->glDrawElements(GL_TRIANGLES, 3 * numFacesSphere, GL_UNSIGNED_INT, 0);
    }

    vaoSphere->release();
    shader->release();
}

// Mouse handlers (optional)
void SceneFluid::mousePressed(const QMouseEvent*, const Camera&) {}
void SceneFluid::mouseMoved(const QMouseEvent*, const Camera&) {}
void SceneFluid::mouseReleased(const QMouseEvent*, const Camera&) {}

// Future SPH methods
void SceneFluid::computeDensityAndPressure() {
    auto &particles = system.getParticles();
    const double w = 315.0 / (64.0 * M_PI * pow(smoothingRadius, 9));

    for (Particle *p1: particles) {
        for (Particle *p2: particles) {
            double d = (p1->pos - p2->pos).norm();
            double t = smoothingRadius * smoothingRadius - d * d;

            if (d < smoothingRadius * smoothingRadius)
                p1->density += particleMass * w * t * t * t;
        }
        p1->pressure = (p1->density - restDensity);
    }
}

void SceneFluid::computeForces() {
}
