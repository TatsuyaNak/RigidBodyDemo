/* Copyright (C) 2025 Tatsuya Nakamura - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the LGPL 2.1 license.
 */

#ifndef COLORSPACEONE_H
#define COLORSPACEONE_H
#include <string>
#include <QGLWidget>
#include <Matrix.h>

// Variables for the 3D view.
struct View
{
    // Camera
	float pos[3];
	float cnt[3];
    float dist;
    float theta;
    float phi;
    // Frustum
    float fovY;
    float nearZ;
    float farZ;
};

// Parameters for control the simulation
struct Params
{
    int worldSpring; 
    int bodySpring;  
    int haveWind;
    int gravity;
    int floor;
    int dispPolys;
    
    int numBodies;
    int stateSize;
    int numVertices;
    int maxPoints;
    
    int step;
    int start;
    int stopped;
};

struct RigidBody
{
    double mass;
    Matrix3x3 Ibody, Ibodyinv;
    
    Vector3d x;
    Matrix3x3 R;
    Vector4d q;
    Vector3d P,L;
    
    Matrix3x3 Iinv;
    Vector3d v,omega;
    
    Vector3d force, torque;
};

struct Particle
{
    double mass;
    Vector3d position;
    Vector3d velocity;
    double radius;
    Particle()
    {
        mass = 0;
        position = Vector3d(0, 0, 0);
        velocity = Vector3d(0, 0, 0);
        radius = 1.0;
    };
    Particle(double m, Vector3d p, Vector3d v, double r) :
    mass(m), position(p), velocity(v), radius(r) {};
};

// Parameters for simulation
struct Simulation
{
    Vector3d wind;
    double windX;
    double windY;
    double windZ;
    
    double mass;
    double viscosity;
    double timeStep;
    double dispTime;
    int timerDelay;
    double coeffofRestitution;
    double time;

    Vector3d gravity;
    RigidBody *bodies;
    Particle *particle;
    Vector3d *init_p;
    double *yfinal;
    Vector4d *init_rbp;

    double kws;
    double kwd;
    double kbs;
    double kbd;
    /* Constatnts for Damping force */
    double kdl;
    double kda;
    
    int *agv_ws;

};

struct WorldSpring
{
    int unsigned bodyIndex;
    int unsigned vertexIndex;
    Vector3d anchor;
    int sw;
    WorldSpring()
    {
        bodyIndex = 256;
        vertexIndex = 256;
        anchor = Vector3d(0.0, 0.0, 0.0);
    }
    WorldSpring( int unsigned B, int unsigned V, Vector3d const &A ) :
    bodyIndex(B), vertexIndex(V), anchor(A) {};
};

struct BodySpring
{
    int unsigned body0Index;
    int unsigned body0VertexIndex;
    int unsigned body1Index;
    int unsigned body1VertexIndex;
    BodySpring()
    {
        body0Index = 256;
        body0VertexIndex = 256;
        body1Index = 256;
        body1VertexIndex = 256;
    }
    BodySpring( int unsigned B0I, int unsigned B0V, int unsigned B1I, int unsigned B1V ) :
    body0Index(B0I), body0VertexIndex(B0V),  body1Index(B1I), body1VertexIndex(B1V) {};
};

// Customized GL Widget with OpenGL
class TnGLWidget : public QGLWidget
{
    Q_OBJECT

public:
    TnGLWidget(QWidget *parent = 0);
    int getWinWidth() const;
    int getWinHeight() const;
    void idle();
    void setWindValue(int value);
    int toggleBodySprings();
    int toggleWorldSprings();
    void resetAll();
    int readParamFile(char *filePath);
    void startSim();
    void stopSim();

protected:
    void initializeGL();
    void resizeGL(int width, int height);
    void paintGL();
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);

private:
    void initVariables();
    std::string paramFilePath;
    int loadParameters();
    void draw();
    void drawSprings();
    void drawPlanes();
    void drawBodies();
    void drawPoints(int index);
    void drawBox(int index);

    void simulate();
    void ode(double *y0, double *yfinal, int array_length, 
             double src_t, double dist_t);
    void Array_to_State(RigidBody *rb, double *y);
    void State_to_Array(RigidBody *rb, double *y);
    Matrix3x3 Star(Vector3d a);
    Matrix3x3 quaternion_to_matrix(const Vector4d q);
    Vector4d quaternion_normalize(Vector4d q);
    void dydt(double y[], double ydot[]);
    void Compute_Force_and_Torque(RigidBody *rb, unsigned int n);
    Vector4d cross_vq(Vector3d a, Vector4d b);
    void ddt_State_to_Array(RigidBody *rb, double *ydot);

    const static int numWorldSprings = 6;
    WorldSpring *worldSprings;
    const static int numBodySprings = 5;
    BodySpring *bodySprings;
    
    GLfloat rotationX;
    GLfloat rotationY;
    GLfloat rotationZ;
    QPoint lastPos;

    Params params;
    Simulation sim;
    
    View view;
    int winWidth;
    int winHeight;
};

#endif
