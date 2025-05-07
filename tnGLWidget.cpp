/* Copyright (C) 2025 Tatsuya Nakamura - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the LGPL 2.1 license.
 */

#include <QtGui>
#if QT_VERSION >= 0x050000
#include <QtWidgets>
#endif
#include <QtOpenGL>
#if defined(Q_OS_MAC)
    #include <GLUT/glut.h>
#elif defined(Q_OS_LINUX)
    #include <GL/glu.h>
#endif

#include "tnGLWidget.h"

TnGLWidget::TnGLWidget(QWidget *parent)
    : QGLWidget(parent)
{
    setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));
    rotationX = -21.0;
    rotationY = -57.0;
    rotationZ = 0.0;

    winWidth = 768;
    winHeight = 768;
    
    view.pos[0] = 0.0;
    view.pos[1] = 360.0;
    view.pos[2] = 600.0;
    view.cnt[0] = view.cnt[2] = 0.0;
    view.cnt[1] = 100.0;

    view.dist = 4.0;
    view.theta = 30.0;
    view.phi = 20.0;
    view.fovY = 30.0;
    view.nearZ = 1.0;
    view.farZ = 1000.0;

    params.worldSpring = TRUE;
    params.bodySpring = TRUE;  
    params.haveWind = FALSE;
    params.gravity = TRUE;
    params.dispPolys = TRUE;
    
    params.start = TRUE;
    params.stopped = TRUE;

    params.numBodies = 6;
    params.stateSize = 13;
    params.numVertices = 8;
    params.maxPoints = 64;    
}

void TnGLWidget::State_to_Array(RigidBody *rb, double *y)
{
    *y++ = rb -> x.x;
    *y++ = rb -> x.y;
    *y++ = rb -> x.z;
    
    *y++ = rb -> q.x;
    *y++ = rb -> q.y;
    *y++ = rb -> q.z;
    *y++ = rb -> q.w;
    
    *y++ = rb -> P.x;
    *y++ = rb -> P.y;
    *y++ = rb -> P.z;
    
    *y++ = rb -> L.x;
    *y++ = rb -> L.y;
    *y++ = rb -> L.z;
}

Matrix3x3 TnGLWidget::quaternion_to_matrix(const Vector4d q)
{
    double s = q.x;
    double vx = q.y;
    double vy = q.z;
    double vz = q.w;
    return Matrix3x3(1.0-2.0*vy*vy-2.0*vz*vz, 2.0*vx*vy-2.0*s*vz, 2.0*vx*vz+2.0*s*vy,
                     2.0*vx*vy + 2.0*s*vz, 1.0-2.0*vx*vx-2.0*vz*vz, 2.0*vy*vz-2.0*s*vx,
                     2.0*vx*vz-2.0*s*vy, 2.0*vy*vz+2.0*s*vx, 1.0-2.0*vx*vx-2.0*vy*vy);
}

Vector4d TnGLWidget::quaternion_normalize(Vector4d q)
{
    double mag = q.norm();
    if (abs(mag)>0.0000001)
        return(Vector4d(q.x/mag, q.y/mag, q.z/mag, q.w/mag));
    else
        return(Vector4d(0.0, 0.0, 0.0, 0.0));
}

void TnGLWidget::Array_to_State(RigidBody *rb, double *y)
{

    rb -> x.x = *y++;
    rb -> x.y = *y++;
    rb -> x.z = *y++;
    
    rb -> q.x = *y++;
    rb -> q.y = *y++;
    rb -> q.z = *y++;
    rb -> q.w = *y++;
    
    rb->R = quaternion_to_matrix(quaternion_normalize(rb->q));
    
    rb -> P.x = *y++;
    rb -> P.y = *y++;
    rb -> P.z = *y++;
    
    rb -> L.x = *y++;
    rb -> L.y = *y++;
    rb -> L.z = *y++;
    
    /* Compute auxilary variables ... */
    
    rb->v = rb->P / rb->mass;
    
    rb->Iinv = rb->R * rb->Ibodyinv * rb->R.transpose();
    
    rb->omega = rb->Iinv * rb->L;
}

Matrix3x3 TnGLWidget::Star(Vector3d a)
{
    return Matrix3x3((double)0.0, (double)(-a.z), (double)(a.y),
                     (double)(a.z), (double)0.0,    (double)(-a.x),
                     (double)(-a.y), (double)(a.x), (double)0.0);
}

void TnGLWidget::stopSim()
{
    params.stopped = TRUE;
}

void TnGLWidget::startSim()
{
    sim.timerDelay = int(0.5 * sim.timeStep * 1000);
    initVariables();
    params.stopped = FALSE;
}

// Load parameter file and reinitialize global parameters

int TnGLWidget::readParamFile(char *filePath)
{
    std::string currentFilePath = paramFilePath;//const char *currentFilePath = (const char *)paramFilePath;
    paramFilePath = std::string(filePath);//TODO
    int result = loadParameters();
    if (result != 1)
    {
        paramFilePath = currentFilePath;
    }
    return result;
}

int TnGLWidget::loadParameters()
{
    FILE *paramfile;
    char buffer[1024];
    int i, num;
    int d;
    
    if (paramFilePath == "")
    {
        fprintf(stderr, "ERROR parameter file is not set.\n");
        return 0;
    }
    const char *charFilePath = paramFilePath.c_str();
    if((paramfile = fopen(charFilePath, "r")) == NULL)
    {
        fprintf(stderr, "ERROR opening parameter file %s\n", charFilePath);
        return 0;
    }

    Simulation copySim = sim;
    double windX, windY, windZ;
    fgets(buffer, 1024, paramfile); 
    if (fscanf(paramfile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
	     &num, &sim.viscosity, &sim.coeffofRestitution,
	     &sim.timeStep, &sim.dispTime,
	     &(windX), &(windY), &(windZ),
	     &(sim.kws), &(sim.kwd),
	     &(sim.kbs), &(sim.kbd),
	     &(sim.kdl), &(sim.kda)) != 14)
    {
        fprintf(stderr, "ERROR reading the 2nd line of the parameter file: %s\n", charFilePath);
        fclose(paramfile);
        sim = copySim;
        return 0;
    }
    sim.windX = windX;
    sim.windY = windY;
    sim.windZ = windZ;
    sim.wind = Vector3d(windX, windY, windZ);

    if ((num < 1)||(num > params.maxPoints))
    {
        fprintf(stderr, "ERROR with the number of balls (%d) in the parameter file %s\n", num, charFilePath);
        fclose(paramfile);
        sim = copySim;
        return 0;
    }
    
    sim.init_p = new Vector3d[params.maxPoints];
    sim.particle = new Particle[params.maxPoints];
    for (i = 0; i < num; i++)
    {
        double velocityX, velocityY, velocityZ;
        double positionX, positionY, positionZ;
        sim.particle[i] = Particle(90, Vector3d(0.0, 50.0, 0.0), Vector3d(40.0, 0.0, 0.0), 1.0);
        d = fscanf(paramfile, "%lf %lf %lf %lf %lf %lf %lf %lf",
                   &(sim.particle[i].mass),
                   &velocityX,
                   &velocityY,
                   &velocityZ,
                   &positionX,
                   &positionY,
                   &positionZ,
                   &(sim.particle[i].radius));
        if(d != 8)
        {
            fprintf(stderr, "ERROR reading parameter file at the #%d rigid body in %s code %d\n", i+1, charFilePath, d);
            fclose(paramfile);
            sim = copySim;
            return 0;
        }
        sim.particle[i].velocity = Vector3d(velocityX, velocityY, velocityZ);    
        sim.particle[i].position = Vector3d(positionX, positionY, positionZ);
        sim.init_p[i] = sim.particle[i].position;
    }
    
    sim.init_rbp = new Vector4d[params.numBodies];
    for (i = 0; i < params.numBodies; i++)
    {
        d = fscanf(paramfile, "%lf %lf %lf %lf",
                   &(sim.init_rbp[i].x),
                   &(sim.init_rbp[i].y),
                   &(sim.init_rbp[i].z),
                   &(sim.init_rbp[i].w));
        if(d != 4)
        {
            fprintf(stderr, "ERROR reading parameter file at the #%d rigid body in %s code %d\n", i+1, charFilePath, d);
            fclose(paramfile);
            sim = copySim;
            return 0;
        }
    }
    return 1;
}

void TnGLWidget::initVariables()
{
    sim.gravity = Vector3d(0.0, -9.8, 0.0);
    
    sim.bodies = new RigidBody[params.numBodies];
    sim.yfinal = new double[params.stateSize * params.numBodies];

    Matrix3x3 ZeroM = Star(Vector3d(0.0, 0.0, 0.0));

    double ixx, iyy, izz;
    int i;
    
    for (i=0; i<params.numBodies; i++)
    {
        //Two bodies
        sim.bodies[i].mass = sim.init_rbp[i].w;
        sim.bodies[i].x.x = sim.init_rbp[i].x;
        sim.bodies[i].x.y = sim.init_rbp[i].y;
        sim.bodies[i].x.z = sim.init_rbp[i].z;
    
        //initial velocity Zero
        sim.bodies[i].v.x= 0.0;
        sim.bodies[i].v.y= 0.0;
        sim.bodies[i].v.z= 0.0;

        //initial angular velocity Zero
        sim.bodies[i].omega.x= 0.0;
        sim.bodies[i].omega.y= 0.0;
        sim.bodies[i].omega.z= 0.0;
        
        //initial force Zero
        sim.bodies[i].force.x = 0.0;
        sim.bodies[i].force.y = 0.0;
        sim.bodies[i].force.z = 0.0;
        //initial torque Zero
        sim.bodies[i].torque.x = 0.0;
        sim.bodies[i].torque.y = 0.0;
        sim.bodies[i].torque.z = 0.0;
        
        //initialize other variables 
        ixx = sim.bodies[i].mass * ((sim.particle[i*8+4].position.y)*(sim.particle[i*8+4].position.y) + (sim.particle[i*8+4].position.z)*(sim.particle[i*8+4].position.z)) /12.0;
        iyy = sim.bodies[i].mass * ((sim.particle[i*8+4].position.z)*(sim.particle[i*8+4].position.z) + (sim.particle[i*8+4].position.x)*(sim.particle[i*8+4].position.x)) /12.0;
        izz = sim.bodies[i].mass * ((sim.particle[i*8+4].position.x)*(sim.particle[i*8+4].position.x) + (sim.particle[i*8+4].position.y)*(sim.particle[i*8+4].position.y)) /12.0;
        
        sim.bodies[i].Ibody = Matrix3x3(ixx, 0.0, 0.0, 0.0, iyy, 0.0, 0.0, 0.0, izz);
        sim.bodies[i].Ibodyinv = sim.bodies[i].Ibody.inv();
        sim.bodies[i].R = Matrix3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        sim.bodies[i].Iinv = ZeroM;
        sim.bodies[i].P = Vector3d(0.0,0.0,0.0);
        sim.bodies[i].L = Vector3d(0.0,0.0,0.0);
        sim.bodies[i].q = Vector4d(1.0,0.0,0.0,0.0);
    }

    //Bodies_to_Array(sim.yfinal);
    for(int i=0; i< params.numBodies; i++)
        State_to_Array(&sim.bodies[i], &sim.yfinal[i * params.stateSize]);
    
    // Initialize springs
    worldSprings = new WorldSpring[numWorldSprings];
    worldSprings[0] = WorldSpring(0,1,Vector3d(-60.0, 200.0, -60.0));
    worldSprings[1] = WorldSpring(1,1,Vector3d(60.0, 200.0, 60.0));
    worldSprings[2] = WorldSpring(2,0,Vector3d(0.0, 200.0, 0.0));
    worldSprings[3] = WorldSpring(3,4,Vector3d(0.0, 150.0, 0.0));
    worldSprings[4] = WorldSpring(4,1,Vector3d(-30.0, 70.0, -30.0));
    worldSprings[5] = WorldSpring(5,1,Vector3d(30.0, 70.0, 30.0));
    
    bodySprings = new BodySpring[numBodySprings];
    bodySprings[0] = BodySpring(2, 3, 3, 7);
    bodySprings[1] = BodySpring(0, 6, 3, 6); 
    bodySprings[2] = BodySpring(1, 7, 3, 3);
    bodySprings[3] = BodySpring(3, 1, 4, 0);
    bodySprings[4] = BodySpring(3, 1, 5, 0);

    sim.agv_ws = new int[numWorldSprings];
    for(i=0; i<numWorldSprings; i++) 
    {
        worldSprings[i].sw=1;
        sim.agv_ws[i]=1;
    }
}

int TnGLWidget::getWinWidth() const
{
    return winWidth;
}

int TnGLWidget::getWinHeight() const
{
    return winHeight;
}

void TnGLWidget::initializeGL()
{
    qglClearColor(Qt::black);
    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_COLOR_MATERIAL);
}

void TnGLWidget::resizeGL(int w, int h)
{

    // Set the 3D view according to the size of the window
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(view.fovY, (double)w/(double)h, view.nearZ, view.farZ);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(view.pos[0], view.pos[1], view.pos[2], view.cnt[0], view.cnt[1], view.cnt[2], 0.0, 1.0, 0.0);
    winWidth = w;
    winHeight = w;
}

void TnGLWidget::simulate()
{

    double y0[params.stateSize * params.numBodies];
    double currentTime = 0.0;
    double targetTime = currentTime + sim.timeStep;
    
    if(params.stopped)
        return;
    
    //integrate
    while(currentTime < targetTime)
    {
        for (int i = 0; i < params.stateSize * params.numBodies; i++)
        {
            y0[i] = sim.yfinal[i];
        }
        ode(y0, sim.yfinal, params.stateSize * params.numBodies, currentTime, targetTime);
        
        currentTime = targetTime;
        for(int i=0; i< params.numBodies; i++)
        {
            Array_to_State(&sim.bodies[i], &sim.yfinal[i * params.stateSize]);
        }

        for (int j=0; j < params.numBodies; j++)
        {
            for (int i = 0; i < params.numVertices; i++)
            {
                sim.particle[j*params.numVertices+i].position = 
                    sim.bodies[j].R * sim.init_p[j*params.numVertices+i];
            }
        }
    }
    if((sim.time / sim.dispTime - (int)(sim.time / sim.dispTime)) < sim.timeStep)
    {
        drawBodies();
    }

}

void TnGLWidget::Compute_Force_and_Torque(RigidBody *rb, unsigned int n)
{
    int i;
    
    rb->force = Vector3d(0.0, 0.0, 0.0);
    rb->torque = Vector3d(0.0, 0.0, 0.0);
    
    if(params.gravity)
    {
        rb->force.x += sim.gravity.x * rb->mass; 
        rb->force.y += sim.gravity.y * rb->mass; 
        rb->force.z += sim.gravity.z * rb->mass; 
    }
    
    if(params.haveWind)
    {
        rb->force.x += sim.viscosity * (sim.wind.x - rb->v.x); 
        rb->force.y += sim.viscosity * (sim.wind.y - rb->v.y); 
        rb->force.z += sim.viscosity * (sim.wind.z - rb->v.z); 
    }
    
    if (params.worldSpring)
    {
        for (i = 0; i< numWorldSprings; i++)
        {
            if (i==n && worldSprings[i].sw==1)
            {
                WorldSpring &springObj = worldSprings[i];
                Vector3d position = sim.particle[params.numVertices*springObj.bodyIndex+springObj.vertexIndex].position;
                
                Vector3d U = position;
                Vector3d VU = rb->v + (rb->omega % U);
                Vector3d spring = -sim.kws * (position + rb->x  - springObj.anchor);
                if (spring.norm()>0.0001)
                {
                    Vector3d dampingForce = -sim.kwd*((VU*spring)/(spring*spring)) * spring;
                    spring = spring + dampingForce;
                }
                rb->force = rb->force + spring;
                rb->torque = rb->torque + U%spring;
            }
        }
    }
    
    if(params.bodySpring)
    {
        for(i = 0; i < numBodySprings; i++)
        {
            BodySpring &springObj = bodySprings[i];
            if ((n==springObj.body0Index)||(n==springObj.body1Index))
            {
                RigidBody rb0 = sim.bodies[springObj.body0Index];
                
                Vector3d position0 = sim.particle[params.numVertices*springObj.body0Index + springObj.body0VertexIndex].position;
                
                Vector3d U0 = position0;
                Vector3d VU0 =  rb0.v + (rb0.omega % U0);
                RigidBody rb1 = sim.bodies[springObj.body1Index];
                
                Vector3d position1 = sim.particle[params.numVertices*springObj.body1Index + springObj.body1VertexIndex].position;
                
                Vector3d U1 = position1;
                Vector3d VU1 =  rb1.v + (rb1.omega % U1);
                
                Vector3d springVector = (position1 + rb1.x) - (position0 + rb0.x);
                Vector3d spring = -sim.kbs * springVector;
                Vector3d relativeVelocity = VU1 - VU0;
                if (springVector.norm()>0.0001)
                {
                    Vector3d dampingForce = (-sim.kbd * (relativeVelocity*springVector)/(springVector*springVector)) * springVector; 
                    
                    spring = spring + dampingForce;
                }
                
                if (n==springObj.body0Index)
                {
                    rb->force = rb->force - spring;
                    rb->torque = rb->torque - U0%spring;
                }
                else if (n==springObj.body1Index)
                {
                    rb->force = rb->force + spring;
                    rb->torque = rb->torque + U1%spring;
                }
            }
        }
        
    }
    
    //damping
    Vector3d FDamping = Vector3d(0.0, 0.0, 0.0);
    Vector3d TDamping = Vector3d(0.0, 0.0, 0.0);
    
    for (i = 0; i<8; i++) 
    {
        Vector3d p = sim.particle[n*8+i].position;
        
        Vector3d r = p ; 
        FDamping = FDamping - sim.kdl * (rb->v + r % rb->omega);
        TDamping = TDamping - sim.kda * r % FDamping;
    }
    rb->force = rb->force + FDamping;
    rb->torque = rb->torque + TDamping;
}

Vector4d TnGLWidget::cross_vq(Vector3d a, Vector4d b)
{
    Vector4d c = Vector4d(0.0, a.x, a.y, a.z);
    double s1 = c.x, s2 = b.x;
    Vector3d v1 = Vector3d(c.y, c.z, c.w);
    Vector3d v2 = Vector3d(b.y, b.z, b.w);
    Vector3d v3 = s1 * v2 + s2 * v1 + v1 % v2;
    return Vector4d(s1*s2-v1*v2, v3.x, v3.y, v3.z);
}

void TnGLWidget::ddt_State_to_Array(RigidBody *rb, double *ydot)
{
    *ydot++ = rb -> v.x;
    *ydot++ = rb -> v.y;
    *ydot++ = rb -> v.z;
    
    Vector4d qdot = 0.5 * cross_vq(rb->omega, rb->q);
    
    *ydot++ = qdot.x;
    *ydot++ = qdot.y;
    *ydot++ = qdot.z;
    *ydot++ = qdot.w;
    
    *ydot++ = rb -> force.x;
    *ydot++ = rb -> force.y;
    *ydot++ = rb -> force.z;
    
    *ydot++ = rb -> torque.x;
    *ydot++ = rb -> torque.y;
    *ydot++ = rb -> torque.z;
}

void TnGLWidget::dydt(double y[], double ydot[])
{
    for(int i=0; i< params.numBodies; i++)
    {
        Array_to_State(&sim.bodies[i], &y[i * params.stateSize]);
    }
    
    for (int i=0; i<params.numBodies; i++)
    {
        Compute_Force_and_Torque(&sim.bodies[i], i);
        ddt_State_to_Array(&sim.bodies[i], &ydot[i * params.stateSize]);
    }

}

void TnGLWidget::ode(double *y0, double *yfinal, int array_length, 
         double src_t, double dist_t)
{
    int i;
    double *s1x, *s1d, *s2x, *s2d, *s3x, *s3d, *s4x, *s4d;
    double deltat = dist_t - src_t;
    
    s1x = new double[array_length];
    for (i=0; i<array_length; i++)
        s1x[i] = y0[i];
    s1d = new double[array_length];
    dydt(s1x, s1d);
    
    s2x = new double[array_length];
    for (i=0; i<array_length; i++)
        s2x[i] = s1x[i] + 0.5 * deltat * s1d[i];
    s2d = new double[array_length];
    dydt(s2x, s2d);
    
    s3x = new double[array_length];
    for (i=0; i<array_length; i++)
        s3x[i] = s2x[i] + 0.5 * deltat * s2d[i];
    s3d = new double[array_length];
    dydt(s3x, s3d);
    
    s4x = new double[array_length];
    for (i=0; i<array_length; i++)
        s4x[i] = s3x[i] + deltat * s3d[i];
    s4d = new double[array_length];
    dydt(s4x, s4d);
    
    for (i=0; i<array_length; i++)
        yfinal[i] = y0[i] + deltat *(s1d[i] + 2.0*s2d[i] + 2.0*s3d[i] + s4d[i])/6.0;    
    
}
void TnGLWidget::idle()
{
    simulate();
    updateGL();
}

int TnGLWidget::toggleBodySprings()
{
    if (params.bodySpring == TRUE)
    {
        params.bodySpring = FALSE;
    }
    else
    {
        params.bodySpring = TRUE;
    }
    return params.bodySpring;
}

int TnGLWidget::toggleWorldSprings()
{
    if (params.worldSpring == TRUE)
    {
        params.worldSpring = FALSE;
    }
    else
    {
        params.worldSpring = TRUE;
    }
    return params.worldSpring;
}

void TnGLWidget::resetAll()
{
    stopSim();
    loadParameters();  
    initVariables();
    setWindValue(0);
    params.bodySpring = TRUE;
    params.worldSpring = TRUE;
    startSim();
}

void TnGLWidget::setWindValue(int value)
{
    if (value == 0)
    {
        params.haveWind = FALSE;
    }
    else
    {
        params.haveWind = TRUE;
        sim.wind = Vector3d(value * 4 * sim.windX, 
                            value * 4 * sim.windY,
                            value * 4 * sim.windZ);
    }
}

void TnGLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    draw();
}

void TnGLWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void TnGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    GLfloat dx = GLfloat(event->x() - lastPos.x()) / width();
    GLfloat dy = GLfloat(event->y() - lastPos.y()) / height();

    if (event->buttons() & Qt::LeftButton) {
        rotationX += 180 * dy;
        rotationY += 180 * dx;
        updateGL();
    } else if (event->buttons() & Qt::RightButton) {
        rotationX += 180 * dy;
        rotationZ += 180 * dx;
        updateGL();
    }
    lastPos = event->pos();
}

void TnGLWidget::drawPlanes()
{
    const int EDGE = 100;
    const float cf_camera_int_x = 0.0;
    const float cf_camera_int_z = 0.0;

    glBegin(GL_POLYGON); 
    glTexCoord2f(0.0, 0.0); // XZ Plane
    glVertex3f(cf_camera_int_x-EDGE, 0.0, cf_camera_int_z-EDGE); /* G0 */
    glTexCoord2f(20.0, 0.0);
    glVertex3f(cf_camera_int_x+EDGE, 0.0, cf_camera_int_z-EDGE); /* G1 */
    glTexCoord2f(20.0, 20.0);
    glVertex3f(cf_camera_int_x+EDGE, 0.0, cf_camera_int_z+EDGE); /* G2 */
    glTexCoord2f(0.0, 20.0);
    glVertex3f(cf_camera_int_x-EDGE, 0.0, cf_camera_int_z+EDGE); /* G3 */
    glEnd();
    
    glColor3f(1.0, 0.5, 0.5); // YX Plane
    glBegin(GL_LINE_LOOP);
    glVertex3f(cf_camera_int_x-EDGE, 0.0, cf_camera_int_z+EDGE);
    glVertex3f(cf_camera_int_x+EDGE, 0.0, cf_camera_int_z+EDGE);
    glVertex3f(cf_camera_int_x+EDGE, EDGE*2.0, cf_camera_int_z+EDGE);
    glVertex3f(cf_camera_int_x-EDGE, EDGE*2.0, cf_camera_int_z+EDGE);
    glEnd();
    
    glColor3f(0.5, 0.5, 1.0); // YZ Plane
    glBegin(GL_LINE_LOOP);
    glVertex3f(cf_camera_int_x-EDGE, 0.0, cf_camera_int_z-EDGE);
    glVertex3f(cf_camera_int_x-EDGE, 0.0, cf_camera_int_z+EDGE);
    glVertex3f(cf_camera_int_x-EDGE, EDGE*2.0, cf_camera_int_z+EDGE);
    glVertex3f(cf_camera_int_x-EDGE, EDGE*2.0, cf_camera_int_z-EDGE);
    glEnd();
}

void TnGLWidget::draw()
{
    
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_POINT);
    glPushMatrix();
    glTranslatef(0.0, 0.0, 0.0);
    glRotatef(rotationX, 1.0, 0.0, 0.0);
    glRotatef(rotationY, 0.0, 1.0, 0.0);
    glRotatef(rotationZ, 0.0, 0.0, 1.0);
    glScalef(1.0, 1.0, 1.0);
    drawPlanes(); 
    if(!params.stopped)
    {
        drawSprings();
        drawBodies();
    }
    glPopMatrix();
    
}

void TnGLWidget::drawSprings()
{
    if (params.worldSpring)
    {
        glPointSize(8.0);
        for (int i = 0; i<numWorldSprings; i++)
        {
            if (worldSprings[i].sw==1)
            {
                glBegin(GL_POINTS);
                Vector3d s = worldSprings[i].anchor;
                glColor3f(0.6, 0.0, 0.8); 
                glVertex3f(s.x, s.y, s.z);
                glEnd();
            }
        }
    }
    if (params.bodySpring)
    {
        glPointSize(8.0);
        for (int i = 0; i<numBodySprings; i++)
        {
            BodySpring &spring = bodySprings[i];
            glBegin(GL_POINTS);
            Vector3d s = sim.particle[params.numVertices*spring.body0Index + spring.body0VertexIndex].position+sim.bodies[spring.body0Index].x;
            glColor3f(0.0, 0.6, 0.8);
            glVertex3f(s.x, s.y, s.z);
            s = sim.particle[params.numVertices*spring.body1Index + spring.body1VertexIndex].position+sim.bodies[spring.body1Index].x;
            glColor3f(0.0, 0.6, 0.8);
            glVertex3f(s.x, s.y, s.z);
            glEnd();
        }
    }
}

void TnGLWidget::drawPoints(int index)
{
    int flag;
    glPointSize(8.0);
    glPushMatrix();
    
    glBegin(GL_POINTS);
    for (int i = 0; i< params.numVertices; i++)
    {
        flag = 0;
        if (flag==0)
        {
            if (i%4==0)
                glColor4f(0.4, 0.8, 0.4, 0.5);
            else if (i%4==1)
                glColor4f(0.0, 0.8, 0.8, 0.5);
            else if (i%4==2)
                glColor4f(0.8, 0.8, 0.0, 0.5);
            else
                glColor4f(0.8, 0.0, 0.0, 0.5);
        }
        glVertex3f(sim.particle[index*8+i].position.x, 
                   sim.particle[index*8+i].position.y, 
                   sim.particle[index*8+i].position.z);
    }
    glEnd();
    glPopMatrix();
    
}

void TnGLWidget::drawBox(int index)
{
    glColor3f(0.0, 0.0, 0.8); //0.1.2.3
    glBegin(GL_QUADS);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(sim.particle[index*8+0].position.x, 
               sim.particle[index*8+0].position.y, 
               sim.particle[index*8+0].position.z);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(sim.particle[index*8+1].position.x, 
               sim.particle[index*8+1].position.y, 
               sim.particle[index*8+1].position.z);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(sim.particle[index*8+2].position.x, 
               sim.particle[index*8+2].position.y, 
               sim.particle[index*8+2].position.z);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(sim.particle[index*8+3].position.x, 
               sim.particle[index*8+3].position.y, 
               sim.particle[index*8+3].position.z);
    glEnd();
    glColor3f(0.4, 0.0, 0.8); //7.6.5.4
    glBegin(GL_QUADS);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(sim.particle[index*8+7].position.x, 
               sim.particle[index*8+7].position.y, 
               sim.particle[index*8+7].position.z);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(sim.particle[index*8+6].position.x, 
               sim.particle[index*8+6].position.y, 
               sim.particle[index*8+6].position.z);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(sim.particle[index*8+5].position.x, 
               sim.particle[index*8+5].position.y, 
               sim.particle[index*8+5].position.z);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(sim.particle[index*8+4].position.x, 
               sim.particle[index*8+4].position.y, 
               sim.particle[index*8+4].position.z);
    glEnd();
    
    glColor3f(0.0, 0.8, 0.0); //0.4.5.1
    glBegin(GL_QUADS);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(sim.particle[index*8+0].position.x, 
               sim.particle[index*8+0].position.y, 
               sim.particle[index*8+0].position.z);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(sim.particle[index*8+4].position.x, 
               sim.particle[index*8+4].position.y, 
               sim.particle[index*8+4].position.z);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(sim.particle[index*8+5].position.x, 
               sim.particle[index*8+5].position.y, 
               sim.particle[index*8+5].position.z);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(sim.particle[index*8+1].position.x, 
               sim.particle[index*8+1].position.y, 
               sim.particle[index*8+1].position.z);
    glEnd();
    glColor3f(0.4, 0.4, 0.4); //2.6.7.3
    glBegin(GL_QUADS);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(sim.particle[index*8+2].position.x, 
               sim.particle[index*8+2].position.y, 
               sim.particle[index*8+2].position.z);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(sim.particle[index*8+6].position.x, 
               sim.particle[index*8+6].position.y, 
               sim.particle[index*8+6].position.z);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(sim.particle[index*8+7].position.x, 
               sim.particle[index*8+7].position.y, 
               sim.particle[index*8+7].position.z);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(sim.particle[index*8+3].position.x, 
               sim.particle[index*8+3].position.y, 
               sim.particle[index*8+3].position.z);
    glEnd();

    glColor3f(0.8, 0.0, 0.0); //0.4.7.3
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(sim.particle[index*8+3].position.x, 
               sim.particle[index*8+3].position.y, 
               sim.particle[index*8+3].position.z);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(sim.particle[index*8+7].position.x, 
               sim.particle[index*8+7].position.y, 
               sim.particle[index*8+7].position.z);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(sim.particle[index*8+4].position.x, 
               sim.particle[index*8+4].position.y, 
               sim.particle[index*8+4].position.z);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(sim.particle[index*8+0].position.x, 
               sim.particle[index*8+0].position.y, 
               sim.particle[index*8+0].position.z);
    glEnd();

    glColor3f(0.8, 0.4, 0.0); //2.6.5.1
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(sim.particle[index*8+1].position.x, 
               sim.particle[index*8+1].position.y, 
               sim.particle[index*8+1].position.z);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(sim.particle[index*8+5].position.x, 
               sim.particle[index*8+5].position.y, 
               sim.particle[index*8+5].position.z);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(sim.particle[index*8+6].position.x, 
               sim.particle[index*8+6].position.y, 
               sim.particle[index*8+6].position.z);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(sim.particle[index*8+2].position.x, 
               sim.particle[index*8+2].position.y, 
               sim.particle[index*8+2].position.z);
    glEnd();
    if (params.worldSpring)
    {
        glPointSize(8.0);
        for (int i = 0; i<numWorldSprings; i++)
        {
            if ((i == index) && (worldSprings[i].sw==1))
            {
                glBegin(GL_POINTS);
                WorldSpring spring = worldSprings[i];
                Vector3d s = spring.anchor;
                Vector3d d = sim.particle[params.numVertices * spring.bodyIndex + spring.vertexIndex].position;
                glColor3f(0.8, 0.8, 0.8); 
                glVertex3f(d.x, d.y, d.z);
                glEnd();
            }
        }
    }
}

void TnGLWidget::drawBodies()
{
    glDepthMask(GL_TRUE);
    
    for (int i=0; i<params.numBodies; i++)
    {
        glPushMatrix();
        glTranslatef(sim.bodies[i].x.x, sim.bodies[i].x.y, sim.bodies[i].x.z);
        if (params.dispPolys)
        {
            glEnable(GL_TEXTURE_2D);
            drawBox(i);
        }
        else
        {
            glDisable(GL_TEXTURE_2D);
            drawPoints(i);
            glEnd();
        }
        glPopMatrix();
    }
}
