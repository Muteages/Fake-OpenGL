//////////////////////////////////////////////////////////////////////
//
//  University of Leeds
//  COMP 5812M Foundations of Modelling & Rendering
//  User Interface for Coursework
//
//  September, 2020
//
//  ------------------------
//  FakeGL.cpp
//  ------------------------
//  
//  A unit for implementing OpenGL workalike calls
//  
///////////////////////////////////////////////////

#include "FakeGL.h"
#include <math.h>
#include "math.hpp"
#include <cmath>

//-------------------------------------------------//
//                                                 //
// CONSTRUCTOR / DESTRUCTOR                        //
//                                                 //
//-------------------------------------------------//

// constructor
FakeGL::FakeGL()
    { // constructor

    //reset state flag
    lightState = 0;
    textureState = 0;
    depthState = 0;
    phongState = 0;

    //viewport
    centerX = 0;
    centerY = 0;
    width = 0;
    height = 0;

    //attributes
    aNormal = {0.0f, 1.0f, 0.0f};
    aColor = {0.0f, 0.0f, 0.0f, 1.0f};
    aTextureCoord = {0.0f, 0.0f, 0.0f};

    //default: no texture
    currentText = nullptr;

    //default mode: replace
    texEevMode = FAKEGL_REPLACE;

    //draw nothing at the beginning
    primitiveType = -1;

    pointSize = 1;
    lineWidth = 1;

    shininess = 1;

    lightPosition = {0.0, 0.0, 0.0};
    //set default rgb(0) and a(1)
    ambientM[0] = ambientM[1] = ambientM[2] = 0.0f;
    diffuseM[0] = diffuseM[1] = diffuseM[2] = 0.0f;
    specularM[0] = specularM[1] = specularM[2] = 0.0f;
    emissionM[0] = emissionM[1] = emissionM[2] = 0.0f;
    //alpha
    ambientM[3] = diffuseM[3] = specularM[3] = emissionM[3] = 1.0f;

    lightAmbient[0] = lightAmbient[1] = lightAmbient[2] = 0.0f;
    lightDiffuse[0] = lightDiffuse[1] = lightDiffuse[2] = 0.0f;
    lightSpecular[0] = lightSpecular[1] = lightSpecular[2] = 0.0f;
    //alpha
    lightAmbient[3] = lightDiffuse[3] = lightSpecular[3] = 1.0f;

    modelViewMatStack.push({});
    projMatStack.push({});
    } // constructor

// destructor
FakeGL::~FakeGL()
    { // destructor

    } // destructor

//-------------------------------------------------//
//                                                 //
// GEOMETRIC PRIMITIVE ROUTINES                    //
//                                                 //
//-------------------------------------------------//

// starts a sequence of geometric primitives
void FakeGL::Begin(unsigned int PrimitiveType)
{ // Begin()

    //clear all queue, prepare for next frame
    vertexQueue.clear();
    rasterQueue.clear();
    fragmentQueue.clear();

    //get primitivetype
    this->primitiveType = PrimitiveType;

    //default: texture is disable
    currentText = nullptr;

} // Begin()

// ends a sequence of geometric primitives
void FakeGL::End()
    { // End()

    //reset
    primitiveType = -1;

    } // End()

// sets the size of a point for drawing
void FakeGL::PointSize(float size)
    { // PointSize()
    pointSize = size;
    } // PointSize()

// sets the width of a line for drawing purposes
void FakeGL::LineWidth(float width)
    { // LineWidth()
    lineWidth = width;
    } // LineWidth()

//-------------------------------------------------//
//                                                 //
// MATRIX MANIPULATION ROUTINES                    //
//                                                 //
//-------------------------------------------------//

// set the matrix mode (i.e. which one we change)   
void FakeGL::MatrixMode(unsigned int whichMatrix)
    { // MatrixMode()
    matrixMode = whichMatrix;
    } // MatrixMode()

// pushes a matrix on the stack
void FakeGL::PushMatrix()
    { // PushMatrix()

    //push the current matrix into the current matrix stack
    Matrix4 matrix;
    matrix.SetIdentity(); 
    switch (matrixMode)
        {
        case FAKEGL_MODELVIEW:
            modelViewMatStack.push(matrix);
            break;
        case FAKEGL_PROJECTION:
            projMatStack.push(matrix);
            break;
        default:
            break;
        }
    } // PushMatrix()

// pops a matrix off the stack
void FakeGL::PopMatrix()
    { // PopMatrix()

    //pop the current matrix from the current matrix stack
    switch(matrixMode)
        {
        case FAKEGL_MODELVIEW:
            modelViewMatStack.pop();
            break;
        case FAKEGL_PROJECTION:
            projMatStack.pop();
            break;
        default:
            break;
        }
    } // PopMatrix()

// load the identity matrix
void FakeGL::LoadIdentity()
    { // LoadIdentity()

    //set the current matrix to the identity matrix
    //set current matrix
    Matrix4* currentMat;

    switch (matrixMode)
    {
    case FAKEGL_MODELVIEW:
        currentMat = &modelViewMatStack.top();
        currentMat->SetIdentity();
    break;
    case FAKEGL_PROJECTION:
        currentMat =  &projMatStack.top();
        currentMat->SetIdentity();
    break;
    }

 } // LoadIdentity()


// multiply by a known matrix in column-major format
void FakeGL::MultMatrixf(const float *columnMajorCoordinates)
    { // MultMatrixf()

    //column-major format
    //[0 4 8  12
    // 1 5 9  13
    // 2 6 10 14
    // 3 7 11 15]

    //use similar iteratation way with [Matrix4::operator *(const Matrix4 &other)]

    //get current matrix
    Matrix4 currentMat = modelViewMatStack.top();

    //start with a zero matrix
    Matrix4* topMat = &modelViewMatStack.top();
    topMat->SetZero();

    // multiply by the known matrix in column-major format
    switch (matrixMode)
        {
        case FAKEGL_MODELVIEW:
            for (int row = 0; row < 4; row++)
                for (int col = 0; col < 4; col++)
                    for (int entry = 0; entry < 4; entry++)
                        modelViewMatStack.top().coordinates[row][col] += currentMat.coordinates[row][entry] * columnMajorCoordinates[entry + col*4];
            break;

        case FAKEGL_PROJECTION:

            for (int row = 0; row < 4; row++)
                for (int col = 0; col < 4; col++)
                    for (int entry = 0; entry < 4; entry++)
                        projMatStack.top().coordinates[row][col] += currentMat.coordinates[row][entry] * columnMajorCoordinates[entry + col*4];
            break;
        default:
            break;
        }

    } // MultMatrixf()


// sets up a perspective projection matrix
void FakeGL::Frustum(float left, float right, float bottom, float top, float zNear, float zFar)
    { // Frustum()
    //creat frustum
    Matrix4 frustumMat;

    //fill value in
    frustumMat[0][0] = (2.0 * zNear) / (right - left);

    frustumMat[1][1] = (2.0 * zNear) / (top - bottom);

    frustumMat[2][0] = (right + left) / (right - left);
    frustumMat[2][1] = (top + bottom) / (top - bottom);
    frustumMat[2][2] = -(zFar + zNear) / (zFar - zNear);
    frustumMat[2][3] = -(2.0f * zNear * zFar) / (zFar - zNear);

    frustumMat[3][2] = -1.0f;

    //multipy by frustumMat
    MultMatrixf(frustumMat.columnMajor().coordinates);

    } // Frustum()

// sets an orthographic projection matrix
void FakeGL::Ortho(float left, float right, float bottom, float top, float zNear, float zFar)
    { // Ortho()

    //creat Ortho
    Matrix4 orthoMat;

     //fill value in

    orthoMat[0][0] = 2.0f / (right - left);

    orthoMat[1][1] = 2.0f / (top - bottom);

    orthoMat[2][2] = -2.0f / (zFar - zNear);

    orthoMat[3][0] = -(right + left) / (right - left);
    orthoMat[3][1] = -(top + bottom) / (top - bottom);
    orthoMat[3][2] = -(zFar + zNear) / (zFar - zNear);
    orthoMat[3][3] = 1.0f;


    //multipy by orthoMat
    MultMatrixf(orthoMat.columnMajor().coordinates);

    } // Ortho()

// rotate the matrix
void FakeGL::Rotatef(float angle, float axisX, float axisY, float axisZ)
    { // Rotatef()

    //creat rotate matrix
    Matrix4 rotateMat;

    //convert angle to radian
    float theta = angle * PI / 180.0;

    //fill the value in matrix
    rotateMat.SetRotation({axisX, axisY, axisZ}, theta);

    //multipy by rotateMat
    MultMatrixf(rotateMat.columnMajor().coordinates);

    } // Rotatef()

// scale the matrix
void FakeGL::Scalef(float xScale, float yScale, float zScale)
    { // Scalef()

    //creat scale matrix
    //[x 0 0 0
    // 0 y 0 0
    // 0 0 z 0
    // 0 0 0 1]

    //creat scale matrix;
    Matrix4 scaleMat;

    //fill the value in matrix
    scaleMat.SetScale(xScale, yScale, zScale);

    //multipy by scaleMat
    MultMatrixf(scaleMat.columnMajor().coordinates);
    } // Scalef()

// translate the matrix
void FakeGL::Translatef(float xTranslate, float yTranslate, float zTranslate)
    { // Translatef()

    //creat translate matrix
    //[1 0 0 x
    // 0 1 0 y
    // 0 0 1 z
    // 0 0 0 1]
    Matrix4 transMat;

    //fill the value in matrix
    transMat.SetTranslation({xTranslate, yTranslate, zTranslate});

    //multipy by transMat
    MultMatrixf(transMat.columnMajor().coordinates);

} // Translatef()

// sets the viewport
void FakeGL::Viewport(int x, int y, int width, int height)
    { // Viewport()

    //width and height
    this->width = width;
    this->height = height;

    //viewport center point
    centerX = x + width / 2;
    centerY = y + height / 2;

    } // Viewport()

//-------------------------------------------------//
//                                                 //
// VERTEX ATTRIBUTE ROUTINES                       //
//                                                 //
//-------------------------------------------------//

// sets colour with floating point
void FakeGL::Color3f(float red, float green, float blue)
    { // Color3f()
    aColor.red = red * 255.0f;
    aColor.green = green * 255.0f;
    aColor.blue = blue * 255.0f;
    aColor.alpha = 255.0f;
    } // Color3f()

// sets material properties
void FakeGL::Materialf(unsigned int parameterName, const float parameterValue)
    { // Materialf()
    if (parameterName & FAKEGL_SHININESS)
    {
        shininess = parameterValue;
    }
    } // Materialf()

void FakeGL::Materialfv(unsigned int parameterName, const float *parameterValues)
    { // Materialfv()
    
    if (parameterName & FAKEGL_AMBIENT)
    {
        ambientM[0] = parameterValues[0];
        ambientM[1] = parameterValues[1];
        ambientM[2] = parameterValues[2];
        ambientM[3] = parameterValues[3];
    }

    if (parameterName & FAKEGL_DIFFUSE)
    {
        diffuseM[0] = parameterValues[0];
        diffuseM[1] = parameterValues[1];
        diffuseM[2] = parameterValues[2];
        diffuseM[3] = parameterValues[3];
    }

    if (parameterName & FAKEGL_SPECULAR)
    {
        specularM[0] = parameterValues[0];
        specularM[1] = parameterValues[1];
        specularM[2] = parameterValues[2];
        specularM[3] = parameterValues[3];
    }

    if (parameterName & FAKEGL_EMISSION)
    {
        emissionM[0] = parameterValues[0];
        emissionM[1] = parameterValues[1];
        emissionM[2] = parameterValues[2];
        emissionM[3] = parameterValues[3];
    }
    
    } // Materialfv()

// sets the normal vector
void FakeGL::Normal3f(float x, float y, float z)
    { // Normal3f()

    aNormal = {x, y, z};

    } // Normal3f()

// sets the texture coordinates
void FakeGL::TexCoord2f(float u, float v)
    { // TexCoord2f()

    aTextureCoord = {u, v, 0};

    } // TexCoord2f()

// sets the vertex & launches it down the pipeline
void FakeGL::Vertex3f(float x, float y, float z)
    { // Vertex3f()

    //creat a vertex
    vertexWithAttributes vertex;

    //fill attributes in
    vertex.position = {x, y, z, 1.0};
    vertex.colour = aColor;
    vertex.normal = aNormal;
    vertex.textureCoord = aTextureCoord;


    //once finished, push it to the queue
    vertexQueue.push_back(vertex);


    //if texture function is enable, get the aTexture; otherwise, set it to null, no texture
    currentText = textureState == 1 ? &aTexture : nullptr;

    //pipeline

    //enter vertexshader stage
    // vertexqueue -> rasterisequeue
    TransformVertex();

    //check if we can enter rasterization stage;
    // rasterisequeue -> fragmentqueue
    if (RasterisePrimitive())
    {
        //processfragement   fragmentShader stage
        ProcessFragment();
    }

    } // Vertex3f()

//-------------------------------------------------//
//                                                 //
// STATE VARIABLE ROUTINES                         //
//                                                 //
//-------------------------------------------------//

// disables a specific flag in the library
void FakeGL::Disable(unsigned int property)
    { // Disable()

    //set value to 0, turn off
    if (property == FAKEGL_LIGHTING) {lightState = 0;}
    if (property == FAKEGL_TEXTURE_2D) {textureState = 0;
                                        currentText = nullptr;}
    if (property == FAKEGL_DEPTH_TEST) {depthState = 0;}
    if (property == FAKEGL_PHONG_SHADING) {phongState = 0;}

    } // Disable()

// enables a specific flag in the library
void FakeGL::Enable(unsigned int property)
    { // Enable()

    //set value to 1, turn on
    if (property == FAKEGL_LIGHTING)   {lightState = 1;}
    if (property == FAKEGL_TEXTURE_2D) {textureState = 1;}
    if(property == FAKEGL_DEPTH_TEST)
    {
       depthState = 1;
       //resize depthBuffer which is the same as framebuffer
       depthBuffer.Resize(frameBuffer.width,frameBuffer.height);
    }

    if (property == FAKEGL_PHONG_SHADING) {phongState = 1;}


    } // Enable()

//-------------------------------------------------//
//                                                 //
// LIGHTING STATE ROUTINES                         //
//                                                 //
//-------------------------------------------------//

// sets properties for the one and only light
void FakeGL::Light(int parameterName, const float *parameterValues)
    { // Light()

    if(parameterName & FAKEGL_AMBIENT)
    {
        lightAmbient[0] = parameterValues[0];
        lightAmbient[1] = parameterValues[1];
        lightAmbient[2] = parameterValues[2];
        lightAmbient[3] = parameterValues[3];
    }
    if(parameterName & FAKEGL_DIFFUSE)
    {
        lightDiffuse[0] = parameterValues[0];
        lightDiffuse[1] = parameterValues[1];
        lightDiffuse[2] = parameterValues[2];
        lightDiffuse[3] = parameterValues[3];
    }
    if(parameterName & FAKEGL_SPECULAR)
    {
        lightSpecular[0] = parameterValues[0];
        lightSpecular[1] = parameterValues[1];
        lightSpecular[2] = parameterValues[2];
        lightSpecular[3] = parameterValues[3];
    }
    if(parameterName & FAKEGL_POSITION)
    {
        //do light calculate in view spacel, so first we need to transform lightposition
        Matrix4 mat = modelViewMatStack.top();
        Cartesian3 lightPos(parameterValues[0], parameterValues[1], parameterValues[2]);
        //convert position to world position
        lightPosition = mat * lightPos;
    }

    } // Light()

//-------------------------------------------------//
//                                                 //
// TEXTURE PROCESSING ROUTINES                     //
//                                                 //
// Note that we only allow one texture             //
// so glGenTexture & glBindTexture aren't needed   //
//                                                 //
//-------------------------------------------------//

// sets whether textures replace or modulate
void FakeGL::TexEnvMode(unsigned int textureMode)
    { // TexEnvMode()

    texEevMode = textureMode;

    } // TexEnvMode()

// sets the texture image that corresponds to a given ID
void FakeGL::TexImage2D(const RGBAImage &textureImage)
    { // TexImage2D()

    aTexture = textureImage;

    } // TexImage2D()

//-------------------------------------------------//
//                                                 //
// FRAME BUFFER ROUTINES                           //
//                                                 //
//-------------------------------------------------//

// clears the frame buffer
void FakeGL::Clear(unsigned int mask)
{   //Clear()

    //iterate pixels and reset each pixel's color to default
    if(mask & FAKEGL_COLOR_BUFFER_BIT)
    {
        for(int row = 0;row<frameBuffer.height;row++)
        {
            for(int col = 0;col<frameBuffer.width;col++)
            {
                frameBuffer[row][col]= clearColor;
            }
        }
    }

    //do the same way to reset depth buffer
    if(mask & FAKEGL_DEPTH_BUFFER_BIT)
    {
        for(int row = 0;row< depthBuffer.height;row++)
        {
            for(int col = 0;col< depthBuffer.width;col++)
            {
                depthBuffer[row][col].alpha= 0.0f;
            }
        }
    }

} // Clear()

// sets the clear colour for the frame buffer
void FakeGL::ClearColor(float red, float green, float blue, float alpha)
    { // ClearColor()

       //need to map from [0,1] to [0,255], multipy 255 here
        clearColor = {red * 255.0f,green* 255.0f,blue* 255.0f,alpha* 255.0f};
    } // ClearColor()

//-------------------------------------------------//
//                                                 //
// MAJOR PROCESSING ROUTINES                       //
//                                                 //
//-------------------------------------------------//

// transform one vertex & shift to the raster queue
void FakeGL::TransformVertex()
    { // TransformVertex()

    //use MVP matrix to complete transform

    while(!vertexQueue.empty())
    {

    //get a vertex from queue
    const vertexWithAttributes & vertex = vertexQueue.front();

    //get current matrix from stack
    Matrix4 modelViewMat = modelViewMatStack.top();
    Matrix4 projMat = projMatStack.top();

    //implement transformation  //projM * viewM * modelM * pos

    //view coordinate
    Homogeneous4 vcsCoord = modelViewMat * vertex.position;

    //clip coordinate
    Homogeneous4 ccsCoord = projMat * vcsCoord;

    //nomralized device coordinate
    Cartesian3 ndcsCoord = ccsCoord.Point();    // x/w y/w z/w

    //add vertex attribute
    screenVertexWithAttributes v;

    //lighting
    //check gouraud Shading state (lighting function)
    //with gouraud shading, we can see some white faint lines, which spilt polygon to triangle
    if (lightState && !phongState)
    {
        //camear position: origin point(0,0,0)
        Cartesian3 cameraPos = {0.0f, 0.0f, 0.0f};

        // vertex position
        Cartesian3 fragPos = vcsCoord.Vector();

        // current normal
        Cartesian3 uNormal = modelViewMat * vertex.normal;

        //calculate light direction
        Cartesian3 lightDir = (lightPosition - fragPos).unit();

        //set light color
        RGBAValue lightAmbientC(lightAmbient[0] * 255.0f,lightAmbient[1] * 255.0f,lightAmbient[2] * 255.0f,lightAmbient[3] * 255.0f);
        RGBAValue lightDiffuseC(lightDiffuse[0] * 255.0f,lightDiffuse[1] * 255.0f,lightDiffuse[2] * 255.0f,lightDiffuse[3] * 255.0f);
        RGBAValue lightSpecularC(lightSpecular[0] * 255.0f,lightSpecular[1] * 255.0f,lightSpecular[2] * 255.0f,lightSpecular[3] * 255.0f);

        //ambient
        RGBAValue ambientColor(ambientM[0] * 255.0f,ambientM[1] * 255.0f,ambientM[2] * 255.0f,ambientM[3] * 255.0f);
        RGBAValue ambient = ambientColor * lightAmbientC;

        //diffuse
        RGBAValue diffColor(diffuseM[0] * 255.0f,diffuseM[1] * 255.0f,diffuseM[2] * 255.0f,diffuseM[3] * 255.0f);
        float diffIntensity = std::max((lightDir.dot(uNormal)), 0.0f);
        RGBAValue diffuse = diffIntensity * diffColor * lightDiffuseC;

        //specular
        RGBAValue specColor(specularM[0] * 255.0f,specularM[1] * 255.0f,specularM[2] * 255.0f,specularM[3] * 255.0f);
        Cartesian3 cameraDir = (cameraPos - fragPos).unit();
        //calculate reflect light
        float x = 2 * lightDir.dot(uNormal);
        Cartesian3 reflect =  (lightDir - uNormal * x).unit();
        float specIntensity = std::pow(std::max(cameraDir.dot(reflect), 0.0f), shininess);
        RGBAValue specular = specIntensity * specColor * lightSpecularC;

        //emission
        RGBAValue emission(emissionM[0] * 255.0f, emissionM[1] * 255.0f, emissionM[2] * 255.0f, emissionM[3] * 255.0f);

        //final color
        RGBAValue color = (ambient + diffuse + specular ) * vertex.colour + emission;
        color.alpha = 255.0f;
        v.colour = color;

    }

    else
    {
        //if the gouraud Shading is disabled. get self color
        v.colour = vertex.colour;
    }


    //NDCS to DCS   c is center point of viewport
    //Xdcs = halfWidth * Xndcs + Xc
    //Ydcs = halfHeight* Yndcs + Yc
    //Zdcs = (Zfar - Znear) / 2 * Zndcs + (Zfar + Znear) / 2
    //OpenGL constraint: Znear = 0, Zfar = 1;  thus Zdcs = Zndcs / 2 + 1 / 2;

    int halfWidth = width / 2;
    int halfHeight = height / 2;
    Cartesian3 dcsCoord;
    dcsCoord.x = centerX + (float)halfWidth * ndcsCoord.x;
    dcsCoord.y = centerY +  (float)halfHeight * ndcsCoord.y;
    dcsCoord.z = (ndcsCoord.z + 1) * 0.5f;

    //get vertex coordinate at device coordinate system (screen)
    v.position = dcsCoord;

    //get uv
    v.textureCoord = vertex.textureCoord;

    //get fragment world position for lighting calculate
    v.fragPos = vcsCoord.Vector();

    // it's better to set normal this way: Normal = transpose(inverse(modelMat)) * aNormal, which can prevent normal distortion
    // but for this assignment, I'd like to make it an easier way.
    v.normal = modelViewMat * vertex.normal;

    //add vertices to queue
    rasterQueue.push_back(v);

    //once finished parse, pop the vertex from queue
    vertexQueue.pop_front();
    }
} // TransformVertex()


// rasterise a single primitive if there are enough vertices on the queue
bool FakeGL::RasterisePrimitive()
{ // RasterisePrimitive()

    //return false if queue is empty
    if (rasterQueue.empty())
    {
        return false;
    }
        //check  geometric primitive type
        switch (primitiveType)
        {
        //rasterise point
        case FAKEGL_POINTS:

            //need enough fragment
            while(rasterQueue.size() > 0)
            {
                screenVertexWithAttributes fragment = rasterQueue.front();
                rasterQueue.pop_front();

                RasterisePoint(fragment);
            }
            return true;
            break;
        //rasterise lines
        case FAKEGL_LINES:

            //need enough fragment
            while(rasterQueue.size() > 1)
            {
                //first fragment
               screenVertexWithAttributes fragment0 = rasterQueue.front();
               rasterQueue.pop_front();
               //second fragment
               screenVertexWithAttributes fragment1 = rasterQueue.front();
               rasterQueue.pop_front();

               RasteriseLineSegment(fragment0,fragment1);
            }
            return true;
            break;

        //rasterise triangles
        case FAKEGL_TRIANGLES:

            //need enough fragment
            while (rasterQueue.size() > 2)
            {
               //first fragment
               screenVertexWithAttributes fragment0 = rasterQueue.front();
               rasterQueue.pop_front();
               //second fragment
               screenVertexWithAttributes fragment1 = rasterQueue.front();
               rasterQueue.pop_front();
               //third fragment
               screenVertexWithAttributes fragment2 = rasterQueue.front();
               rasterQueue.pop_front();

               RasteriseTriangle(fragment0, fragment1, fragment2);
            }
            return true;
            break;

        default:
            return false;
            break;
        }

} // RasterisePrimitive()


// rasterises a single point
void FakeGL::RasterisePoint(screenVertexWithAttributes &vertex0)
    { // RasterisePoint()

    //here we use a similar way which is provided in RasteriseTriangle function to rasterise point
     //create a temp fragment
    fragmentWithAttributes tempFrag;

    // compute a bounding box
    // clipping will happen in the raster loop proper
    // set the point range
    float minX = vertex0.position.x - (pointSize / 2.0);
    float minY = vertex0.position.y - (pointSize / 2.0);
    float maxX = vertex0.position.x + (pointSize / 2.0);
    float maxY = vertex0.position.y + (pointSize / 2.0);

    // loop through the pixels in the bounding box
    for (tempFrag.row = minY; tempFrag.row <= maxY; tempFrag.row++)
        { // per row
        // skip parsing if it is outside the frame buffer
        if (tempFrag.row < 0) continue;
        if (tempFrag.row >= frameBuffer.height) continue;
        for (tempFrag.col = minX; tempFrag.col <= maxX; tempFrag.col++)
            { // per pixel
            // skip parsing if it is outside the frame buffer
            if (tempFrag.col < 0) continue;
            if (tempFrag.col >= frameBuffer.width) continue;

            tempFrag.colour = vertex0.colour;
            tempFrag.normal = vertex0.normal;
            tempFrag.textureCoord = vertex0.textureCoord;
            tempFrag.fragPos = vertex0.fragPos;

            if(depthTest(vertex0.position.x,vertex0.position.y, vertex0.position.z))
            {
                if(depthState) {depthBuffer[(int)vertex0.position.y][(int)vertex0.position.x].alpha = vertex0.position.z * 255;}
                //add this fragment to the queue
                fragmentQueue.push_back(tempFrag);
            }

            } // per pixel
        } // per row
    } // RasterisePoint()


// rasterises a single line segment
void FakeGL::RasteriseLineSegment(screenVertexWithAttributes &vertex0, screenVertexWithAttributes &vertex1)
 { // RasteriseLineSegment()

    //using Digital Differential Analyzer algorithm
    for (int w = 0; w < lineWidth; w++)
    {

    //the line is from vertex0 to vertex1
    float x0 = vertex0.position.x + w;
    float x1 = vertex1.position.x + w;
    float y0 = vertex0.position.y + w;
    float y1 = vertex1.position.y + w;

    //total increment
    int dx = x1 - x0;
    int dy = y1 - y0;

    int steps;
    float xIncrement, yIncrement;
    float x = x0, y = y0;

    if (abs(dx) > abs(dy))
    {
        steps = abs(dx);
    }
    else
    {
        steps = abs(dy);
    }

    xIncrement = float(dx) / float(steps);
    yIncrement = float(dy) / float(steps);

    fragmentWithAttributes fragment;


    //for start point and end point, we use their own attributes, because we don't need to calculate their attributes by interplation algorithm.
    //start point
    fragment.row = y0;
    fragment.col = x0;
    fragment.colour = vertex0.colour;
    fragment.normal = vertex0.normal;
    fragment.textureCoord = vertex0.textureCoord;
    fragment.fragPos = vertex0.fragPos;

    //if pass depth test, add the fragment to queue
    if(depthTest(x0,y0,vertex0.position.z))
    {
        if(depthState) {depthBuffer[(int)y0][(int)x0].alpha = vertex0.position.z * 255;}
        fragmentQueue.push_back(fragment);
    }


    // interpolate the points between v0 and v1
    for(int k = 0; k < steps; ++k)
    {
        x += xIncrement;
        y += yIncrement;

        fragment.row = y;
        fragment.col = x;

        //delta [0, 1]
        float delta = float(k) / float(steps);

        //interpolate attributes
        fragment.colour = Math::lerp(vertex0.colour,vertex1.colour,delta);
        fragment.normal = Math::lerp(vertex0.normal,vertex1.normal,delta);
        fragment.textureCoord = Math::lerp(vertex0.textureCoord,vertex1.textureCoord,delta);
        fragment.fragPos = Math::lerp(vertex0.fragPos,vertex1.fragPos,delta);
        Cartesian3 pos =  Math::lerp(vertex0.position,vertex1.position,delta);

        //depth test
        if(depthTest(x,y,pos.z))
        {
            if(depthState) {depthBuffer[(int)y][(int)x].alpha = pos.z * 255;}

            //add this fragment to the queue
            fragmentQueue.push_back(fragment);
        }
    }

    //end point
    fragment.row = y1;
    fragment.col = x1;
    fragment.colour = vertex1.colour;
    fragment.normal = vertex1.normal;
    fragment.textureCoord = vertex1.textureCoord;
    fragment.fragPos = vertex1.fragPos;

    if(depthTest(x1,y1, vertex1.position.z))
    {
        if(depthState) {depthBuffer[(int)y1][(int)x1].alpha = vertex1.position.z * 255;}
        //add vertex1 to the queue
        fragmentQueue.push_back(fragment);
    }
    }

 } // RasteriseLineSegment()


// rasterises a single triangle
void FakeGL::RasteriseTriangle(screenVertexWithAttributes &vertex0, screenVertexWithAttributes &vertex1, screenVertexWithAttributes &vertex2)
    { // RasteriseTriangle()

    // compute a bounding box that starts inverted to frame size
    // clipping will happen in the raster loop proper
    float minX = frameBuffer.width, maxX = 0.0;
    float minY = frameBuffer.height, maxY = 0.0;
    
    // test against all vertices
    if (vertex0.position.x < minX) minX = vertex0.position.x;
    if (vertex0.position.x > maxX) maxX = vertex0.position.x;
    if (vertex0.position.y < minY) minY = vertex0.position.y;
    if (vertex0.position.y > maxY) maxY = vertex0.position.y;
    
    if (vertex1.position.x < minX) minX = vertex1.position.x;
    if (vertex1.position.x > maxX) maxX = vertex1.position.x;
    if (vertex1.position.y < minY) minY = vertex1.position.y;
    if (vertex1.position.y > maxY) maxY = vertex1.position.y;
    
    if (vertex2.position.x < minX) minX = vertex2.position.x;
    if (vertex2.position.x > maxX) maxX = vertex2.position.x;
    if (vertex2.position.y < minY) minY = vertex2.position.y;
    if (vertex2.position.y > maxY) maxY = vertex2.position.y;

    // now for each side of the triangle, compute the line vectors
    Cartesian3 vector01 = vertex1.position - vertex0.position;
    Cartesian3 vector12 = vertex2.position - vertex1.position;
    Cartesian3 vector20 = vertex0.position - vertex2.position;

    // now compute the line normal vectors
    Cartesian3 normal01(-vector01.y, vector01.x, 0.0);  
    Cartesian3 normal12(-vector12.y, vector12.x, 0.0);  
    Cartesian3 normal20(-vector20.y, vector20.x, 0.0);  

    // we don't need to normalise them, because the square roots will cancel out in the barycentric coordinates
    float lineConstant01 = normal01.dot(vertex0.position);
    float lineConstant12 = normal12.dot(vertex1.position);
    float lineConstant20 = normal20.dot(vertex2.position);

    // and compute the distance of each vertex from the opposing side
    float distance0 = normal12.dot(vertex0.position) - lineConstant12;
    float distance1 = normal20.dot(vertex1.position) - lineConstant20;
    float distance2 = normal01.dot(vertex2.position) - lineConstant01;

    // if any of these are zero, we will have a divide by zero error
    // but notice that if they are zero, the vertices are collinear in projection and the triangle is edge on
    // we can render that as a line, but the better solution is to render nothing.  In a surface, the adjacent
    // triangles will eventually take care of it
    if ((distance0 == 0) || (distance1 == 0) || (distance2 == 0))
        return; 

    // create a fragment for reuse
    fragmentWithAttributes rasterFragment;

    // loop through the pixels in the bounding box
    for (rasterFragment.row = minY; rasterFragment.row <= maxY; rasterFragment.row++)
        { // per row
        // this is here so that clipping works correctly
        if (rasterFragment.row < 0) continue;
        if (rasterFragment.row >= frameBuffer.height) continue;
        for (rasterFragment.col = minX; rasterFragment.col <= maxX; rasterFragment.col++)
            { // per pixel
            // this is also for correct clipping
            if (rasterFragment.col < 0) continue;
            if (rasterFragment.col >= frameBuffer.width) continue;
            
            // the pixel in cartesian format
            Cartesian3 pixel(rasterFragment.col, rasterFragment.row, 0.0);
            
            // right - we have a pixel inside the frame buffer AND the bounding box
            // note we *COULD* compute gamma = 1.0 - alpha - beta instead
            float alpha = (normal12.dot(pixel) - lineConstant12) / distance0;           
            float beta = (normal20.dot(pixel) - lineConstant20) / distance1;            
            float gamma = (normal01.dot(pixel) - lineConstant01) / distance2;           

            // now perform the half-plane test
            if ((alpha < 0.0) || (beta < 0.0) || (gamma < 0.0))
                continue;

            // compute attributes

            rasterFragment.colour = alpha * vertex0.colour + beta * vertex1.colour + gamma * vertex2.colour;
            rasterFragment.normal = alpha * vertex0.normal + beta * vertex1.normal + gamma * vertex2.normal;
            rasterFragment.textureCoord = alpha * vertex0.textureCoord + beta * vertex1.textureCoord + gamma * vertex2.textureCoord;
            rasterFragment.fragPos = alpha * vertex0.fragPos + beta * vertex1.fragPos + gamma * vertex2.fragPos;
            Cartesian3 pos = alpha * vertex0.position + beta * vertex1.position + gamma * vertex2.position;

            //depth test
            if(depthTest(rasterFragment.col, rasterFragment.row, pos.z))
            {
                if(depthState)
                {
                    depthBuffer[rasterFragment.row][rasterFragment.col].alpha = pos.z * 255.0f;
                }

                //add this fragment to the queue
                    fragmentQueue.push_back(rasterFragment);
            }
            } // per pixel
        } // per row
    } // RasteriseTriangle()

// process a single fragment
void FakeGL::ProcessFragment()
   { // ProcessFragment()

    while (!fragmentQueue.empty())
    {
        //get the fragment from queue
        fragmentWithAttributes & frag = fragmentQueue.front();
        if(frag.row <= frameBuffer.height && frag.col <= frameBuffer.width && frag.row >= 0 && frag.col >= 0)
        {

            if(texEevMode == FAKEGL_REPLACE)
            {
                frameBuffer[frag.row][frag.col] = FragmentShader(frag);

            }
            if(texEevMode == FAKEGL_MODULATE)
            {
                //texture modulate : multipy the texture color and the color itself
                frameBuffer[frag.row][frag.col] = FragmentShader(frag) * frag.colour;
            }
            if(phongState)
            {
                frameBuffer[frag.row][frag.col] = FragmentShader(frag);
            }
        }

        //once finishing, pop the fragment from queue, and be ready for next fragment
        fragmentQueue.pop_front();
    }
} // ProcessFragment()


//depth test method for reuse
bool FakeGL::depthTest(float x,float y, float z)
{

    if(depthState){
          if( depthBuffer[(int)y][(int)x].alpha >=  z * 255.0f)
          {
             return false;
          }
      }
     //default
     return true;
}

// simulates FragmentShader stage
RGBAValue FakeGL:: FragmentShader(fragmentWithAttributes & frag)
    {//FragmentShader()

    //implement lighting
    //check Phong Shading state
    //with phong shading, it will display smoother at the edge of triangles
    if (lightState && phongState)
    //if(phongState)
    {

        //camear position: origin point(0,0,0)
        Cartesian3 cameraPos = {0.0f, 0.0f, 0.0f};

        // current fragment position
        Cartesian3 fragPos = frag.fragPos;

        // current normal (normalized)
        Cartesian3 uNormal = frag.normal.unit();

        //calculate light direction (normalized)
        Cartesian3 lightDir = (lightPosition - fragPos).unit();

        //set light color
        RGBAValue lightAmbientC(lightAmbient[0] * 255.0f,lightAmbient[1] * 255.0f,lightAmbient[2] * 255.0f,lightAmbient[3] * 255.0f);
        RGBAValue lightDiffuseC(lightDiffuse[0] * 255.0f,lightDiffuse[1] * 255.0f,lightDiffuse[2] * 255.0f,lightDiffuse[3] * 255.0f);
        RGBAValue lightSpecularC(lightSpecular[0] * 255.0f,lightSpecular[1] * 255.0f,lightSpecular[2] * 255.0f,lightSpecular[3] * 255.0f);

        //ambient
        RGBAValue ambientColor(ambientM[0] * 255.0f,ambientM[1] * 255.0f,ambientM[2] * 255.0f,ambientM[3] * 255.0f);
        RGBAValue ambient = ambientColor * lightAmbientC;

        //diffuse
        RGBAValue diffColor(diffuseM[0] * 255.0f,diffuseM[1] * 255.0f,diffuseM[2] * 255.0f,diffuseM[3] * 255.0f);
        float diffIntensity = std::max((lightDir.dot(uNormal)), 0.0f);
        RGBAValue diffuse = diffIntensity * diffColor * lightDiffuseC;

        //specular
        RGBAValue specColor(specularM[0] * 255.0f,specularM[1] * 255.0f,specularM[2] * 255.0f,specularM[3] * 255.0f);
        Cartesian3 cameraDir = (cameraPos - fragPos).unit();
        //calculate reflect light
        float x = 2.0f * lightDir.dot(uNormal);
        Cartesian3 reflect = lightDir - uNormal * x;
        float specIntensity = std::pow(std::max(cameraDir.dot(reflect), 0.0f), shininess);
        RGBAValue specular = specIntensity * specColor * lightSpecularC;

        //emission
        RGBAValue emission(emissionM[0] * 255.0f, emissionM[1] * 255.0f, emissionM[2] * 255.0f, emissionM[3] * 255.0f);
        RGBAValue Result = (ambient + diffuse + specular) * frag.colour + emission;
        Result.alpha = 255.0f;
        return Result;

    }

    //return original color if the texture is not used
    if(currentText == nullptr)
    {
        return frag.colour;
    }

    int u = std::min((long)(frag.textureCoord.x* (currentText->width - 1)),currentText->width- 1);
    int v = std::min((long)((frag.textureCoord.y) * (currentText->height - 1)),currentText->height- 1);
    return aTexture[v][u];

 }// FragmentShader()

// standard routine for dumping the entire FakeGL context (except for texture / image)
std::ostream &operator << (std::ostream &outStream, FakeGL &fakeGL)
    { // operator <<
    outStream << "=========================" << std::endl;
    outStream << "Dumping FakeGL Context   " << std::endl;
    outStream << "=========================" << std::endl;


    outStream << "-------------------------" << std::endl;
    outStream << "Vertex Queue:            " << std::endl;
    outStream << "-------------------------" << std::endl;
    for (auto vertex = fakeGL.vertexQueue.begin(); vertex < fakeGL.vertexQueue.end(); vertex++)
        { // per matrix
        outStream << "Vertex " << vertex - fakeGL.vertexQueue.begin() << std::endl;
        outStream << *vertex;
        } // per matrix


    outStream << "-------------------------" << std::endl;
    outStream << "Raster Queue:            " << std::endl;
    outStream << "-------------------------" << std::endl;
    for (auto vertex = fakeGL.rasterQueue.begin(); vertex < fakeGL.rasterQueue.end(); vertex++)
        { // per matrix
        outStream << "Vertex " << vertex - fakeGL.rasterQueue.begin() << std::endl;
        outStream << *vertex;
        } // per matrix


    outStream << "-------------------------" << std::endl;
    outStream << "Fragment Queue:          " << std::endl;
    outStream << "-------------------------" << std::endl;
    for (auto fragment = fakeGL.fragmentQueue.begin(); fragment < fakeGL.fragmentQueue.end(); fragment++)
        { // per matrix
        outStream << "Fragment " << fragment - fakeGL.fragmentQueue.begin() << std::endl;
        outStream << *fragment;
        } // per matrix


    return outStream;
    } // operator <<

// subroutines for other classes
std::ostream &operator << (std::ostream &outStream, vertexWithAttributes &vertex)
    { // operator <<
    std::cout << "Vertex With Attributes" << std::endl;
    std::cout << "Position:   " << vertex.position << std::endl;
    std::cout << "Colour:     " << vertex.colour << std::endl;



    return outStream;
    } // operator <<

std::ostream &operator << (std::ostream &outStream, screenVertexWithAttributes &vertex) 
    { // operator <<
    std::cout << "Screen Vertex With Attributes" << std::endl;
    std::cout << "Position:   " << vertex.position << std::endl;
    std::cout << "Colour:     " << vertex.colour << std::endl;

    return outStream;
    } // operator <<

std::ostream &operator << (std::ostream &outStream, fragmentWithAttributes &fragment)
    { // operator <<
    std::cout << "Fragment With Attributes" << std::endl;
    std::cout << "Row:        " << fragment.row << std::endl;
    std::cout << "Col:        " << fragment.col << std::endl;
    std::cout << "Colour:     " << fragment.colour << std::endl;

    return outStream;
    } // operator <<


    
    
