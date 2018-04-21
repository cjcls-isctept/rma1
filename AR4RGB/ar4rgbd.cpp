/*
 * Mixed Reality and Applications
 *
 * Pedro Santana
 * 2013-2017 ISCTE-IUL
 *
 *  Assumed frames of reference:
 *  - PCL: z-blue (into the scene), x-red (to the right), y-green (downwards)
 *  - OSG: z (upwards), x (to the left), y (out of the scene)
 *
 */

#include <osg/Camera>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osg/Texture2D>
#include <osg/MatrixTransform>
#include <osg/GraphicsContext>
#include <osg/Depth>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/PositionAttitudeTransform>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osgGA/GUIEventHandler>
#include <osg/ShapeDrawable>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>	
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <string>
#include <algorithm>
#include <sstream>

static const char* textureVertexSource = {
    "varying vec3 normal;\n"
    "void main()\n"
    "{\n"
    "    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;\n"
    "}\n"
    //test
    //test 412211212
};

static const char* ballVertexSource = {
    "void main()\n"
    "{\n"
    "    gl_TexCoord[0] = gl_MultiTexCoord0;\n"
    "    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;\n"
    "}\n"
};

static const char* textureFramentSource = {
    "uniform sampler2D texture;\n"
    "uniform sampler2D textureDepth;\n"
    "void main()\n"
    "{\n"
    "   vec2 uv = vec2(gl_FragCoord.x/640.0, gl_FragCoord.y/480.0);\n"
    "   gl_FragColor = texture2D(texture, uv);\n"
    "   gl_FragDepth = (texture2D(textureDepth, uv)[2]);\n"
    "}\n"
};

static const char* ballFragmentSource = {
    "uniform sampler2D texture;\n"
    "void main()\n"
    "{\n"
    "    gl_FragColor = texture2D(texture, gl_TexCoord[0].st);\n"
    "    gl_FragDepth = (1.0/gl_FragCoord.w)/1000.0;\n"
    "}\n"
};

class BallCallback : public osg::NodeCallback
{
public:
    BallCallback(osg::PositionAttitudeTransform *_shadowTransf) :
    shadowTransf(_shadowTransf){}

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
protected:
    osg::PositionAttitudeTransform* shadowTransf;
};

void BallCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{

    osg::PositionAttitudeTransform* ballTransf = static_cast<osg::PositionAttitudeTransform*>(node);
    osg::Vec3 v = ballTransf->getPosition();
    shadowTransf->setPosition(osg::Vec3(v.x(), v.y(), v.z()-3));
}


void	CreateBall(osg::ref_ptr<osg::PositionAttitudeTransform> ballTransf,
                   osg::ref_ptr<osg::PositionAttitudeTransform> shadowTransf)
{
    osg::ref_ptr<osg::Shader> vertShader2 =
    new osg::Shader( osg::Shader::VERTEX, ballVertexSource );
    osg::ref_ptr<osg::Shader> fragShader2 =
    new osg::Shader( osg::Shader::FRAGMENT, ballFragmentSource );
    osg::ref_ptr<osg::Program> program2 = new osg::Program;
    program2->addShader( fragShader2.get() );
    program2->addShader( vertShader2.get() );

    osg::Texture2D *ballTexture = new osg::Texture2D;
    ballTexture->setDataVariance(osg::Object::DYNAMIC);
    ballTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    ballTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    ballTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP);
    ballTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);
    osg::Image *ballImage = osgDB::readImageFile("../Data/PlatonicSurface_Color.jpg");
    ballTexture->setImage(ballImage);

    osg::ref_ptr<osg::Node> ballNode = osgDB::readNodeFile("../Data/soccer_ball.obj");
    ballNode->getOrCreateStateSet()->setMode( GL_LIGHT1, osg::StateAttribute::ON );
    osg::StateSet *ballStateSet = ballNode->getOrCreateStateSet();
    ballStateSet->setTextureAttributeAndModes(0, ballTexture, osg::StateAttribute::ON);
    ballStateSet->setAttributeAndModes( program2.get() );
    ballStateSet->addUniform(new osg::Uniform("texture", 0 ));

    osg::Image *shadowImage = osgDB::readImageFile("../Data/shadow.png");
    osg::ref_ptr<osg::Texture2D> shadowTexture = new osg::Texture2D(shadowImage);
    osg::ref_ptr<osg::StateSet> shadowStateSet = new osg::StateSet();
    shadowStateSet->setTextureAttributeAndModes(0, shadowTexture, osg::StateAttribute::ON);
    shadowStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    osg::ref_ptr<osg::BlendFunc> texture_blending_function = new osg::BlendFunc();
    shadowStateSet->setAttributeAndModes(texture_blending_function.get(), osg::StateAttribute::ON);
    shadowStateSet->setAttributeAndModes( program2.get() );
    shadowStateSet->addUniform(new osg::Uniform("texture", 0 ));

    osg::ref_ptr<osg::Drawable> shadowQuad =
    osg::createTexturedQuadGeometry( osg::Vec3(0.5f, 0.5f, 0.0f), osg::Vec3(-1.0f, 0.0f, 0.0f), osg::Vec3(0.0f, -1.0f, 0.0f) );
    shadowQuad->setStateSet(shadowStateSet.get());
    osg::ref_ptr<osg::Geode> shadowGeode = new osg::Geode;
    shadowGeode->addDrawable( shadowQuad.get() );

    shadowTransf->addChild(shadowGeode);
    shadowTransf->setPosition(osg::Vec3(0,-100,2));
    shadowTransf->setDataVariance( osg::Object::DYNAMIC );
    shadowTransf->setScale(osg::Vec3(8,8,2));

    ballTransf->addChild(ballNode);
    ballTransf->setScale(osg::Vec3(.05,.05,.05));
    ballTransf->setPosition(osg::Vec3(0,-100,5));
    ballTransf->setAttitude(osg::Quat(0.0f, osg::Y_AXIS, 0.0f, osg::Z_AXIS, 0.0f, osg::X_AXIS));
    ballTransf->setDataVariance( osg::Object::DYNAMIC );

    static_cast<osg::Node*>(ballTransf)->setUpdateCallback( new BallCallback(shadowTransf.get()) );

}


class BallController : public osgGA::GUIEventHandler
{
public:
    BallController( osg::PositionAttitudeTransform* node, bool *_collidedLeft, bool *_collidedRight,
                   bool *_collidedFront, bool *_collidedBack, bool *_collidedBelow)
    : _ball(node), collidedLeft(_collidedLeft),
    collidedRight(_collidedRight), collidedFront(_collidedFront), collidedBack(_collidedBack), collidedBelow(_collidedBelow) {}
    virtual bool handle( const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter& aa );
protected:
    osg::ref_ptr<osg::PositionAttitudeTransform> _ball;
    bool *collidedLeft, *collidedRight, *collidedFront, *collidedBack, *collidedBelow;
};

bool BallController::handle( const osgGA::GUIEventAdapter& ea,
                            osgGA::GUIActionAdapter& aa )
{
    if ( !_ball )
        return false;
    
    //39.

    switch ( ea.getEventType() )
    {
            
        case osgGA::GUIEventAdapter::KEYDOWN:
            switch ( ea.getKey() )
        {
            case 'a': case 'A':
                //40.

                break;
            case 'd': case 'D':

                break;
            case 'w': case 'W':
                //41.

                break;
            case 's': case 'S':

                break;

            default:
                break;
        }
            
            //42.


            break;
        default:
            break;
    }
    return false;
    
}

void estimateCameraPose(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in, double *pitch, double *roll, double *height)
{

    //3.

    
    //4.


    //5.


    //6.


    //7.


    //8.A.

    
    //8.


    //8.B.


    //9.

}


void rotatePointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in,
                      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated,
                      double camera_pitch, double camera_roll, double camera_height)
{

    //11.


    //12.


    //13.


}

void createImageFromPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in, unsigned char* data, unsigned char* dataDepth)
{
    long index1, index2;
    for (int row=0;row<cloud_in->height;row++)
    {
        for (int col=0;col<cloud_in->width;col++)
        {
            index1 = 3*(row*cloud_in->width + col);
            index2 = (cloud_in->height-row-1)*cloud_in->width + col;

            //15.


            //16.


        }
    }
}

void createVirtualCameras(osg::ref_ptr<osg::Camera> camera1, osg::ref_ptr<osg::Camera> camera2, osg::ref_ptr<osg::Geode> orthoTexture,
                          double camera_pitch, double camera_roll, double camera_height)
{
    
    //25.


    //26.


    //27.


    //28.


    //29.


    //30.


    //31.


    //32.

}


void	createOrthoTexture(osg::ref_ptr<osg::Geode> orthoTextureGeode,
                           unsigned char* data, unsigned char* dataDepth, int width, int height)
{
    //18.


    //19.


    //20.


    //21.


    //22.

}




void detectCollisions(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ballPath, osg::ref_ptr<osg::PositionAttitudeTransform> ballTransf,
                      pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree,
                      bool *collidedLeft, bool *collidedRight, bool *collidedFront, bool *collidedBack, bool *collidedBelow)
{

    std::vector<int> search_indexes;
    std::vector<float> search_radiuses;

    //45.


    //46.


    //47.


    //48.


    //49.


}

int main(int argsc, char** argsv){
    
    // create a point cloud to keep track of the ball's path
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ballPath (new pcl::PointCloud<pcl::PointXYZRGBA>);
    ballPath->height = 1;

    // ball dynamics state variables
    bool collidedLeft = false, collidedRight = false, collidedFront = false, collidedBack = false, collidedBelow = false;
    
    // reading point cloud
    //1.


    // estimate the camera pose w.r.t. to ground plane
    //2.


    // rotate point cloud so as to align the ground plane with a virtual's ground plane
    //10.

    // go through the point cloud and generate a RGB image and a range image
    //14.


    // create a texture from the RGB image and use depth data to fill the z-buffer
    //17.


    // create an orthographic camera imaging the texture and a perspective camera based on the camera's pose and intrinsics
    //24.


    // add the two cameras to the scene's root node
    //33.


    // create a dynamic ball node alongside its shadow
    //36.


    // run a controller to allow the user to control the ball with the keyboard
    //37.


    // force the perspective camera look at the ball and the shadow
    //38.


    // create a root's viewer
    //34.


    //38.A.
    
    // create a kdtree from the point cloud in order to speed up collision detection
    //44.


    //35.



    if (ballPath->size() > 0)
    {
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZRGBA> ("Out/ball_path.pcd", *ballPath, false);
    }

    
}

