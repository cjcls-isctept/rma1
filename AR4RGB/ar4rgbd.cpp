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

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>   // std::cout
#include <string>     // std::string, std::to_string
#include <algorithm>
#include <sstream>

pcl::visualization::PCLVisualizer *viewer;
//double dim_vec[4];

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

void pp_callback (const pcl::visualization::PointPickingEvent& event, void*)
{

    int idx = event.getPointIndex ();

    if (idx == -1)

    return;

    pcl::PointXYZRGBA picked_pt;

    event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);

    std::cout << "Picked point: " << picked_pt.x << " " << picked_pt.y << " " << picked_pt.z << std::endl;

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
    osg::Vec3 v = _ball->getPosition();

    osg::Quat q = _ball->getAttitude();

    switch ( ea.getEventType() )
    {
            
        case osgGA::GUIEventAdapter::KEYDOWN:
            switch ( ea.getKey() )
        {
            case 'a': case 'A':
                //40.
                if (!(*collidedLeft)){
                    v.x() += 1.0;

                    q *= osg::Quat(0.2, osg::Y_AXIS, 0.0, osg::X_AXIS, 0.0, osg::Z_AXIS);

                }
                break;
            case 'd': case 'D':
                if (!(*collidedLeft)){
                    v.x() -= 1.0;

                    q *= osg::Quat(0.2, osg::Y_AXIS, 0.0, osg::X_AXIS, 0.0, osg::Z_AXIS);

                }
                break;
            case 'w': case 'W':
                //41.
                if (!(*collidedFront)){

                    v.y() -= 1.0;

                    q *= osg::Quat(0.2, osg::X_AXIS, 0.0, osg::Y_AXIS, 0.0, osg::Z_AXIS);

                }

                break;
            case 's': case 'S':
                if (!(*collidedFront)){

                    v.y() += 1.0;

                    q *= osg::Quat(0.2, osg::X_AXIS, 0.0, osg::Y_AXIS, 0.0, osg::Z_AXIS);

                }
                break;

            default:
                break;
        }
            
            //42.
            _ball->setPosition(v);

            _ball->setAttitude(q);


            break;
        default:
            break;
    }
    return false;
    
}

void estimateCameraPose(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in, double *pitch, double *roll, double *height)
{

    //3.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_centre (new pcl::PointCloud<pcl::PointXYZRGBA>);

    cloud_centre->height=1; int step = 2; long index;

    for (int row=0;row<cloud_in->height/1;row+=step){

        for (int col=cloud_in->width/2-300;col<cloud_in->width/2+300;col+=step){

            index = (cloud_in->height-row-1)*cloud_in->width + col;

            cloud_centre->points.push_back(cloud_in->points[index]);

            cloud_centre->width++;
        }
    }
    
    //4.
    pcl::PCDWriter writer;

    writer.write<pcl::PointXYZRGBA> ("Out/cloud_centre.pcd", *cloud_centre, false);


    //5.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

    seg.setModelType (pcl::SACMODEL_PLANE);

    seg.setMethodType (pcl::SAC_RANSAC);

    seg.setMaxIterations (10000);

    seg.setDistanceThreshold (0.05);

    seg.setInputCloud (cloud_centre);

    seg.segment (*inliers, *coefficients);

    //6.
    double c_a = coefficients->values[0];

    double c_b = coefficients->values[1];

    double c_c = coefficients->values[2];

    double c_d = coefficients->values[3];

    std::cout << "Coefficients a: " << c_a << " b: " << c_b << " c: " << c_c << " d: " << c_d << "." << std::endl;


    //7.
    double norm = sqrt(c_a*c_a+c_b*c_b+c_c*c_c);

    c_a = c_a/norm;

    c_b = c_b/norm;

    c_c = c_c/norm;

    std::cout << "Coefficients a: " << c_a << " b: " << c_b << " c: " << c_c << " d: " << c_d << " norm: " << norm << std::endl;

    //8.A.
    if (c_c<0){
    
    c_a*=-1; c_b*=-1; c_c*=-1;   
    
    }
    
    //8.
    *pitch = asin(c_c);
    
    *roll = -acos(c_b/cos(*pitch));
    
    *height = fabs(c_d);

    //8.B.
    if (c_a<0)
    
    (*roll) *= -1;    
    
    

    //9.
    std::cout << "Camera pitch: " << *pitch * 180/M_PI << " [deg]; Camera roll: " << *roll * 180/M_PI << " [deg]." << std::endl;
   
    std::cout << "Camera height: " << *height << std::endl;

}

//extrai o maior cluster da nuvem de input, devolvendo-o
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_clusterization(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_table_only,
                                                             pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster_selected){

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_temporary (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);

    tree->setInputCloud (cloud_table_only);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;

    ec.setClusterTolerance (0.02);

    ec.setMinClusterSize (10);

    ec.setMaxClusterSize (25000);

    ec.setSearchMethod (tree);

    ec.setInputCloud (cloud_table_only);

    ec.extract (cluster_indices);

    // Picking the largest cluster (in principle the object of interest)
    int max_size = 0;

    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster_selected (new pcl::PointCloud<pcl::PointXYZRGBA>);

    for (int i=0; i<cluster_indices.size(); i++){

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);

        for (int j=0; j < cluster_indices[i].indices.size(); j++){

            cloud_cluster->points.push_back (cloud_table_only->points[cluster_indices[i].indices[j]]);

        }

       // if(allClusters==true) return cluster_indices;

        cloud_cluster->width = cloud_cluster->points.size ();

        cloud_cluster->height = 1;

        cloud_cluster->is_dense = true;

        if (cloud_cluster->size() > max_size){

            max_size = cloud_cluster->size();

            cloud_temporary = cloud_cluster;

        }

    }
    cloud_cluster_selected = cloud_temporary;
      // std::cout<< " dasdaasd "<<cloud_cluster_selected->size() <<" dasdasdasdasd"<< std::endl;
    //std::vector<pcl::PointIndices> c;
    //c[0]=cloud_cluster_selected;
    return cloud_cluster_selected;

}


//extrai os clusters todos
void getAllClusters(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_table_only, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>& clusters_vector ){

    //cout << "Esta a entrar no metidoodsaasd " << std::endl;

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);

    tree->setInputCloud (cloud_table_only);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;

    ec.setClusterTolerance (0.02);

    ec.setMinClusterSize (10);

    ec.setMaxClusterSize (25000);

    ec.setSearchMethod (tree);

    ec.setInputCloud (cloud_table_only);

    ec.extract (cluster_indices);

    //cout << "TAMANHO  " << cluster_indices.size() << " TAMANHO!! " << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster_selected (new pcl::PointCloud<pcl::PointXYZRGBA>);

    for (int i=0; i<cluster_indices.size(); i++){

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);

        for (int j=0; j < cluster_indices[i].indices.size(); j++){

            cloud_cluster->points.push_back (cloud_table_only->points[cluster_indices[i].indices[j]]);

        }

        cloud_cluster->width = cloud_cluster->points.size ();

        cloud_cluster->height = 1;

        cloud_cluster->is_dense = true;

       // std::cout <<"  antes "<< std::endl;

       // std::cout <<"tamanho do cluster  " << i << "  " << cloud_cluster->points.size () << " capacity" << std::endl;

        clusters_vector.push_back(cloud_cluster);

        //std::cout <<"  DEPOIS "<< std::endl;


    }
    //cout <<"LOLOL" << cluster_indices.at(0) << "LOLLOL";
    //return cloud_cluster;


}



//devolve a nuvem com o plano dominante da nuvem
void dominant_plane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated_2, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_table_only ){
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

    seg.setOptimizeCoefficients (true);

    seg.setModelType (pcl::SACMODEL_PLANE);

    seg.setMethodType (pcl::SAC_RANSAC);

    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud_rotated_2);

    seg.segment (*inliers, *coefficients);

    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_table_only (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

    extract.setInputCloud (cloud_rotated_2);

    extract.setIndices (inliers);

    extract.setNegative (false);

    extract.filter (*cloud_table_only);

    //return cloud_table_only;
}

//corta a nuvem em ordem a Y(vertical), de forma a que sobre a nuvem com a mesa
void trim_table(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated_2 ){
    //filtrar pontos com altura(y) superior a um treshold
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated_2 (new pcl::PointCloud<pcl::PointXYZRGBA>);

    //pegar nos pontos da nuvem rodada que tao no segmento vertical da mesa, e colocar numa nuvem nova cloud_rotated_2
    for (size_t i = 0; i < cloud_rotated->points.size(); i++)
        if (cloud_rotated->points[i].y < 0.05)
            cloud_rotated_2->points.push_back (cloud_rotated->points[i]);

    cloud_rotated_2->width = 1;
    cloud_rotated_2->height = cloud_rotated_2->points.size();


}


double calculate_average(double a, double b){
    return (a+b)/2;
}



pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clean_n_rotate_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in,
                                                             pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated,
                                                             double camera_pitch, double camera_roll, double camera_height){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_voxelised (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;

    voxel_grid.setInputCloud (cloud_in);

    voxel_grid.setLeafSize (0.01, 0.01, 0.01);

    voxel_grid.filter (*cloud_voxelised);


    Eigen::Affine3f t1 = pcl::getTransformation (0.0, -camera_height, 0.0, 0.0, 0.0, 0);

    Eigen::Affine3f t2 = pcl::getTransformation (0.0, 0.0, 0.0, -camera_pitch, 0.0, 0.0);

    Eigen::Affine3f t3 = pcl::getTransformation (0.0, 0.0, 0.0, 0.0, 0.0, -camera_roll);


    pcl::transformPointCloud(*cloud_voxelised, *cloud_rotated, t1*t2*t3);
    std::cout << "taansadmanho 1" << cloud_rotated->points.size() << "  tamanho " << std::endl;


}


//calcula as dimensoes da mesa
void calculate_dimensions(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<double>& dim_vec){
    double var_Zmax = 0;
    double var_Zmin = 1;

    double var_Xmax = 0;
    double var_Xmin = 1;



    for (size_t i = 0; i < cloud->points.size(); i++){
        if(cloud->points[i].z>var_Zmax)
            var_Zmax = cloud->points[i].z;

        if(cloud->points[i].z<var_Zmin)
            var_Zmin = cloud->points[i].z;

        if(cloud->points[i].x>var_Xmax)
            var_Xmax = cloud->points[i].x;

        if(cloud->points[i].x<var_Xmin)
            var_Xmin = cloud->points[i].x;


    }


    std::cout << "   Output comprimentoZ: " << var_Zmax - var_Zmin << std::endl;
    std::cout << "   Output comprimentoX: " << var_Xmax - var_Xmin << std::endl;
    //cout << (calculate_average(var_z1max, var_z2max)-(calculate_average(var_z3min, var_z4min)));
    //cout << "   Output larguraX: "; // prints Output sentence on screen
    //cout << (calculate_average(var_x1max, var_x2max)-(calculate_average(var_x3min, var_x4min)));

    //devolve as dimensões, na forma xmin,xmax,zmin, zmax

    //dim_vec[4] = new double[4];
    //std::vector<double> dim_vec (4);
    dim_vec.push_back(var_Xmin);
    dim_vec.push_back(var_Xmax);
    dim_vec.push_back(var_Zmin);
    dim_vec.push_back(var_Zmax);



    //return dim_vec;


}



void obtainTableDimensions(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in,
                      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated,
                      double camera_pitch, double camera_roll, double camera_height, std::vector<double>& dime_vec)
{

    //new code

    //roda e voxiliza a nuvem, diminuindo possivel excessiva densidade, e roda, alinhando o tampo da mesa com o eixo Z
    clean_n_rotate_cloud(cloud_in, cloud_rotated, camera_pitch, camera_roll, camera_height);

    //trim para obter segmento com a mesa, e eliminar grande parte do "lixo"
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated_2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
    trim_table(cloud_rotated, cloud_rotated_2);
    //fim do trim


    //plano dominante
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_table_only (new pcl::PointCloud<pcl::PointXYZRGBA>);
    dominant_plane(cloud_rotated_2, cloud_table_only);
    //fim do plano dominante

    //segmentacao da nuvem para obter só a mesa,
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_table_cleaned (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //cloud_clusterization(cloud_table_only, cloud_table_cleaned);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_table_cleaned=cloud_clusterization(cloud_table_only, cloud_table_cleaned);

    //13.
    pcl::PCDWriter writer;

    writer.write<pcl::PointXYZRGBA> ("Out/out_rotated2.pcd", *cloud_table_only, false);


  //  std::cout << "TAMANHO DA NUVEM  " << cloud_table_cleaned->size() << " FFFFF" << std::endl;
    calculate_dimensions(cloud_table_cleaned, dime_vec);
  //   std::cout<< " dasdaasd "<< dime_vec.capacity() <<" dasdasdasdasd"<< std::endl;



    //cálculo das dimensões
    //return calculate_dimensions(cloud_table_cleaned);



}

void colorAverage(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>& clusters_vector){

    //

    for(int i=0; i<clusters_vector.size(); i++){

        int r=0;
        int g=0;
        int b=0;

        for(int j=0; j<clusters_vector[i]->points.size(); j++){

                //std::cout<< " valor double ou int "<< (int)clusters_vector[0]->points[1].r <<" dasdasdasdasd"<< std::endl;
            r=r+(int)clusters_vector[i]->points[j].r;
            g=g+(int)clusters_vector[i]->points[j].g;
            b=b+(int)clusters_vector[i]->points[j].b;

        }
        r=r/clusters_vector[i]->points.size();
        g=g/clusters_vector[i]->points.size();
        b=b/clusters_vector[i]->points.size();

        for(int j=0; j<clusters_vector[i]->points.size(); j++){

                //std::cout<< " valor double ou int "<< (int)clusters_vector[0]->points[1].r <<" dasdasdasdasd"<< std::endl;
            clusters_vector[i]->points[j].r=r;
            clusters_vector[i]->points[j].g=g;
            clusters_vector[i]->points[j].b=b;


        }



    }

}





//faz trim de acordo com o valor do Y dado em val_y
//se trim_above=true, ignora tudo o que está pra cima de val_y
//se trim_above=false, ignora tudo o que está para baixo de val_y
void trimCloudByYX( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_trimmed,
                                                       double val_y, std::vector<double>& dimensions, bool trim_above){

    //filtrar pontos com altura(y) superior a um treshold
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_trimmed (new pcl::PointCloud<pcl::PointXYZRGBA>);

    //pegar nos pontos da nuvem rodada que tao no segmento vertical da mesa, e colocar numa nuvem nova cloud_rotated_2
    for (size_t i = 0; i < cloud_rotated->points.size(); i++){
        if ( cloud_rotated->points[i].x > dimensions.at(0) && cloud_rotated->points[i].x < dimensions.at(1)
                && cloud_rotated->points[i].z > dimensions.at(2) && cloud_rotated->points[i].z < dimensions.at(3)){
            if(trim_above && cloud_rotated->points[i].y < val_y){

                cloud_trimmed->points.push_back (cloud_rotated->points[i]);

            }if(trim_above==false && cloud_rotated->points[i].y > val_y){

                cloud_trimmed->points.push_back (cloud_rotated->points[i]);

            }
        }

    }

    cloud_trimmed->width = 1;
    cloud_trimmed->height = cloud_trimmed->points.size();

}





void tableObjectsClusterization(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in, double camera_pitch,
                                double camera_roll, double camera_height, std::vector<double>& dimensions, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>& clusters_vector){

    //std::vector<double> table_dim=obtainTableDimensions();

    //roda e ajusta a nuvem para ficar melhor, e devove a cloud_rotated
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated (new pcl::PointCloud<pcl::PointXYZRGBA>);

    clean_n_rotate_cloud(cloud_in, cloud_rotated, camera_pitch, camera_roll, camera_height);

    //fazer trim à nuvem, de acordo com a altura (Y), para eliminar a mesa e restar tudo o que está a cima do tampo

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_trimmed_by_Y (new pcl::PointCloud<pcl::PointXYZRGBA>);
    trimCloudByYX(cloud_rotated ,cloud_trimmed_by_Y ,-0.05, dimensions, true);

    pcl::PCDWriter writer;

    //esta cloud tem os objetos da mesa
    writer.write<pcl::PointXYZRGBA> ("Out/out_trimmed_cloud.pcd", *cloud_trimmed_by_Y, false);

    //
   // std::vector<pcl::PointIndices> cluster_indices= getAllClusters(cloud_trimmed_by_Y);

    // vetor clusters_vector irá conter os clusters dos objetos na mesa

    getAllClusters(cloud_trimmed_by_Y, clusters_vector);


  //  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster1 = clusters_vector[0];

    colorAverage(clusters_vector);


    //escrever num .pcd os clusters dos objetos na mesa, separadas. cada um numa nuvem
    std::string path="Out/out_cloud_obj";
    std::string extension=".pcd";
    for(int i=0; i<clusters_vector.size(); i++){
        std::string iteration=boost::to_string(i);
        std::string file= path+iteration;
        file=file+extension;
        writer.write<pcl::PointXYZRGBA> (file, *clusters_vector[i], false);
    }


}

void trimm_other_than_table(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_trimmed,
                            std::vector<double>& dimensions){



}

void handDetection(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in, std::vector<double>& dimensions,
                   double camera_pitch, double camera_roll, double camera_height){

    pcl::PCDWriter writer;
    //roda e ajusta a nuvem para ficar melhor, e devove a cloud_rotated
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated (new pcl::PointCloud<pcl::PointXYZRGBA>);
    clean_n_rotate_cloud(cloud_in, cloud_rotated , camera_pitch, camera_roll, camera_height);



    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_trimmed (new pcl::PointCloud<pcl::PointXYZRGBA>);

  //  std::cout << "taansadmanho " << cloud_in->points.size() << "  tamanho " << std::endl;
    std::cout << "taansadmanho 2" << cloud_rotated->points.size() << "  tamanho " << std::endl;
    //trimm_other_than_table(cloud_rotated, cloud_trimmed, dimensions);
    //é false, para ignorar tudo o que esta para baixo de 0.05
    trimCloudByYX(cloud_rotated ,cloud_trimmed,-0.05 , dimensions, false);


    //writer.write<pcl::PointXYZRGBA> ("Out/out_trimmed_cloud_test.pcd", *cloud_trimmed, true);



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
            data[index1] = cloud_in->points[index2].r;

            data[index1+1] = cloud_in->points[index2].g;

            data[index1+2] = cloud_in->points[index2].b;


            //16.
            //se ele nao tiver info da depth do ponto, mete a zeros
            if (isnan(cloud_in->points[index2].x)){

                dataDepth[index1] = dataDepth[index1+1] = dataDepth[index1+2] = 0;

            }else{
            //se nao, vai la buscar os calor xyz
                dataDepth[index1] = ((cloud_in->points[index2].x+2)/6.0)*255.0;

                dataDepth[index1+1] = ((cloud_in->points[index2].y+2)/6.0)*255.0;

                dataDepth[index1+2] = (cloud_in->points[index2].z/10.0)*255.0;

            }


        }
    }
}




void createVirtualCameras(osg::ref_ptr<osg::Camera> camera1, osg::ref_ptr<osg::Camera> camera2, osg::ref_ptr<osg::Geode> orthoTexture,
                          double camera_pitch, double camera_roll, double camera_height)
{
    
    //25.
    camera1->setCullingActive( false );

    camera1->setClearMask( 0 );

    camera1->setAllowEventFocus( false );

    camera1->setReferenceFrame( osg::Camera::ABSOLUTE_RF );

    camera1->setRenderOrder( osg::Camera::POST_RENDER, 0);

    camera1->setProjectionMatrix( osg::Matrix::ortho(0.0, 1.0, 0.0, 1.0, 0.5 , 1000) );

    camera1->addChild( orthoTexture.get() );

    osg::StateSet* ss = camera1->getOrCreateStateSet();

    ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    //26.
    camera2->setCullingActive( false );

    camera2->setClearMask( 0 );

    camera2->setAllowEventFocus( false );

    camera2->setReferenceFrame( osg::Camera::ABSOLUTE_RF );

    camera2->setRenderOrder( osg::Camera::POST_RENDER, 1);

    camera2->setProjectionMatrixAsPerspective(50, 640./480., 0.5, 1000);

    //27.
    osg::Matrixd cameraRotation2;

    cameraRotation2.makeRotate(osg::DegreesToRadians(camera_pitch*180/M_PI), osg::Vec3(1,0,0),

                                                         osg::DegreesToRadians(0.0), osg::Vec3(0,1,0),

                                                         osg::DegreesToRadians(0.0), osg::Vec3(0,0,1));

    //28.
    osg::Matrixd cameraRotation3;

    cameraRotation3.makeRotate(osg::DegreesToRadians(0.0), osg::Vec3(1,0,0),

    osg::DegreesToRadians(camera_roll*180/M_PI), osg::Vec3(0,1,0),

    osg::DegreesToRadians(0.0), osg::Vec3(0,0,1));

    //29.
    osg::Matrixd cameraTrans;

    cameraTrans.makeTranslate(0, 0, camera_height*100.0);

    //30.
    osg::Matrix matrix_view;

    matrix_view.makeLookAt(osg::Vec3(0, 0, 0), osg::Vec3(0, -1, 0), osg::Vec3(0, 0, 1));

    osg::Matrixd cameraRotation;

    cameraRotation.makeRotate(osg::DegreesToRadians(180.0), osg::Vec3(0,0,1), osg::DegreesToRadians(-90.0), osg::Vec3(1,0,0), osg::DegreesToRadians(0.0), osg::Vec3(0,1,0) );


    //31.
    camera2->setViewMatrix(osg::Matrix::inverse(cameraRotation*cameraRotation3*cameraRotation2*cameraTrans));

    //32.
    camera1->setViewport(0, 0, 640, 480);

    camera2->setViewport(0, 0, 640, 480);
}


void	createOrthoTexture(osg::ref_ptr<osg::Geode> orthoTextureGeode,
                           unsigned char* data, unsigned char* dataDepth, int width, int height)
{
    //18.    

    osg::Image *image = new osg::Image;

   osg::Image *imageDepth = new osg::Image;

   image->setOrigin(osg::Image::TOP_LEFT);

   imageDepth->setOrigin(osg::Image::TOP_LEFT);

   image->setImage(width, height, 1 , GL_RGB, GL_RGB, GL_UNSIGNED_BYTE,data, osg::Image::NO_DELETE);

   imageDepth->setImage(width, height, 1 , GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, dataDepth, osg::Image::NO_DELETE);

   //19.
   osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;

   osg::ref_ptr<osg::Texture2D> texture2 = new osg::Texture2D;

   texture->setImage( image );

   texture2->setImage(imageDepth);

    //20.
   osg::ref_ptr<osg::Drawable> quad = osg::createTexturedQuadGeometry( osg::Vec3(), osg::Vec3(1.0f, 0.0f, 0.0f), osg::Vec3(0.0f, 1.0f, 0.0f) );

   quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture.get() );

   quad->getOrCreateStateSet()->setTextureAttributeAndModes(1, texture2.get() );

    //21.
   osg::ref_ptr<osg::Shader> vertShader = new osg::Shader( osg::Shader::VERTEX, textureVertexSource );

   osg::ref_ptr<osg::Shader> fragShader =  new osg::Shader( osg::Shader::FRAGMENT, textureFramentSource );

   osg::ref_ptr<osg::Program> program = new osg::Program;

   program->addShader( fragShader.get() );

   program->addShader( vertShader.get() );

    //22.
   quad->setDataVariance(osg::Object::DYNAMIC);

   quad->getOrCreateStateSet()->setAttributeAndModes( program.get() );

   quad->getOrCreateStateSet()->addUniform(new osg::Uniform("texture", 0 ));

   quad->getOrCreateStateSet()->addUniform(new osg::Uniform("textureDepth", 1 ));

   orthoTextureGeode->addDrawable( quad.get() );
}



void detectCollisions(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ballPath, osg::ref_ptr<osg::PositionAttitudeTransform> ballTransf,
                      pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree,
                      bool *collidedLeft, bool *collidedRight, bool *collidedFront, bool *collidedBack, bool *collidedBelow)
{

    std::vector<int> search_indexes;
    std::vector<float> search_radiuses;

    //45.
    pcl::PointXYZRGBA ball;

    ball.x = -ballTransf->getPosition().x()/100.f;

    ball.y = -ballTransf->getPosition().z()/100.f;

    ball.z = -ballTransf->getPosition().y()/100.f;

    //46.
    pcl::PointXYZRGBA ball_view;

    ball_view.x = -ballTransf->getPosition().x()/100.f;

    ball_view.y = ballTransf->getPosition().z()/100.f;

    ball_view.z = ballTransf->getPosition().y()/100.f;

    ballPath->width++;

    ballPath->push_back(ball_view);
    //47.
    pcl::PointXYZRGBA ballNeighborPoint;

    ballNeighborPoint.x = ball.x - 0.05;

    ballNeighborPoint.y = ball.y - 0.05;

    ballNeighborPoint.z = ball.z;

    //48.
kdtree->radiusSearch (ballNeighborPoint, 0.05, search_indexes, search_radiuses);

    //49.
if (search_indexes.size() == 0)

*collidedLeft = false;

else

*collidedLeft = true;

}



int main(int argsc, char** argsv){
    
    // create a point cloud to keep track of the ball's path
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ballPath (new pcl::PointCloud<pcl::PointXYZRGBA>);
    ballPath->height = 1;

    // ball dynamics state variables
    bool collidedLeft = false, collidedRight = false, collidedFront = false, collidedBack = false, collidedBelow = false;
    
    // reading point cloud
    //1.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in_arg2 (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in_arg3 (new pcl::PointCloud<pcl::PointXYZRGBA>);



//DEBUG
    //../Data/Clouds/seq_3/background_no_objs_3/129976532142.pcd ../Data/Clouds/seq_3/background_objs_3/129976479791.pcd

    //argsv[1]="../Data/Clouds/seq_3/background_no_objs_3/129976532142.pcd";
    //argsv[2]="../Data/Clouds/seq_3/background_objs_3/129976479791.pcd";

//DEBUG





    if (pcl::io::loadPCDFile (argsv[1], *cloud_in) == -1)  {

        PCL_ERROR ("Couldn't read the PCD file 1 \n");

        return (-1);

    }
    if (pcl::io::loadPCDFile (argsv[2], *cloud_in_arg2) == -1)  {

        PCL_ERROR ("Couldn't read the PCD file 2 \n");

        return (-1);

    }
    if (pcl::io::loadPCDFile (argsv[3], *cloud_in_arg3) == -1)  {

        PCL_ERROR ("Couldn't read the PCD file 3 \n");

        return (-1);

    }



    /*
    //added code
    pcl::PointClouda<pcl::PointXYZ>::Ptr cloud_voxelised (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> vg;

    vg.setInputCloud (cloud_in);

    vg.setLeafSize (0.01f, 0.01f, 0.01f);

    vg.filter (*cloud_voxelised);

    // two pointers that will be directed to the point clouds we want to render in red and in blue
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_red_thin (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blue_thick (new pcl::PointCloud<pcl::PointXYZ>);


    // -- VIZUALISATION POINTERS (change it to visualise the various point clouds)
    //0.
    cloud_red_thin = cloud_in;

    cloud_blue_thick = cloud_voxelised;

    */
    //end of added code



    // estimate the camera pose w.r.t. to ground plane
    //2.
    double camera_pitch, camera_roll, camera_height;

    estimateCameraPose(cloud_in, &camera_pitch, &camera_roll, &camera_height);








    // rotate point cloud so as to align the ground plane with a virtual's ground plane
    //10.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated (new pcl::PointCloud<pcl::PointXYZRGBA>);



    std::vector<double> dimensions;

    obtainTableDimensions(cloud_in, cloud_rotated ,camera_pitch, camera_roll, camera_height, dimensions);
    //std::cout << "LOLO " << dimensions[0] << "   WOWOWO" << std::endl;
    //cout << "PRIMEIRO VALOROROROROROROR   ";
    //cout << obtainTableDimensions(cloud_in, cloud_rotated, camera_pitch, camera_roll, camera_height).at(1);
    //cout << "   FIMMMMMMMMMMMMMMM";
//test

    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters_vector;

    tableObjectsClusterization(cloud_in_arg2, camera_pitch, camera_roll,
                               camera_height, dimensions, clusters_vector);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster1 = clusters_vector[0];


    handDetection(cloud_in_arg3, dimensions, camera_pitch, camera_roll, camera_height);








//test



    // go through the point cloud and generate a RGB image and a range image
    //14.
    int size = cloud_in->height*cloud_in->width*3;

   unsigned char* data = (unsigned char*)calloc(size,sizeof(unsigned char));

   unsigned char* dataDepth = (unsigned char*)calloc(size,sizeof(unsigned char));

   createImageFromPointCloud(cloud_in, data, dataDepth);




    // create a texture from the RGB image and use depth data to fill the z-buffer
    //17.
   osg::ref_ptr<osg::Geode> orthoTextureGeode = new osg::Geode;

   createOrthoTexture(orthoTextureGeode, data, dataDepth, cloud_in->width, cloud_in->height);

    // create an orthographic camera imaging the texture and a perspective camera based on the camera's pose and intrinsics
    //24.
   osg::ref_ptr<osg::Camera> camera1 = new osg::Camera;

   osg::ref_ptr<osg::Camera> camera2 = new osg::Camera;

   createVirtualCameras(camera1, camera2, orthoTextureGeode, camera_pitch, camera_roll, camera_height);



    // add the two cameras to the scene's root node
    //33.
   osg::ref_ptr<osg::Group> root = new osg::Group;

   root->addChild( camera1.get() );

   root->addChild( camera2.get() );


    // create a dynamic ball node alongside its shadow
    //36.
   osg::ref_ptr<osg::PositionAttitudeTransform> ballTransf = new osg::PositionAttitudeTransform;

   osg::ref_ptr<osg::PositionAttitudeTransform> shadowTransf = new osg::PositionAttitudeTransform;

  CreateBall(ballTransf, shadowTransf);


    // run a controller to allow the user to control the ball with the keyboard
    //37.
   osg::ref_ptr<BallController> ctrler =  new BallController( ballTransf.get(), &collidedLeft, &collidedRight, &collidedFront, &collidedBack, &collidedBelow);

    // force the perspective camera look at the ball and the shadow
    //38.
   camera2->addChild( ballTransf );

    camera2->addChild( shadowTransf );


    // create a root's viewer
    //34.
    osgViewer::Viewer viewer;

    viewer.setUpViewInWindow(0, 0, 640, 480);

    viewer.setSceneData( root.get() );

    //38.A.
    viewer.addEventHandler( ctrler.get() );

    // create a kdtree from the point cloud in order to speed up collision detection
    //44.
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
   kdtree->setInputCloud (cloud_rotated);

    //35.
   while (!viewer.done() ){
   //43.
//detectCollisions(ballPath, ballTransf, kdtree, &collidedLeft, &collidedRight, &collidedFront, &collidedBack, &collidedBelow);
   viewer.frame();

   }


    if (ballPath->size() > 0)
    {
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZRGBA> ("Out/ball_path.pcd", *ballPath, false);
    }

    
}

