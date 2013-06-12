
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "stdafx.h"
#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Eigen>

#include <boost/thread.hpp> 
#include <boost/array.hpp> 
#include <boost/format.hpp> 

#include "KinectSensor.h"

#define sleep(x) Sleep((x) * 1000) 

// comment-toggle
const int DEVICE_NUMBER = 2;
std::vector<KinectSensor::Ptr> sensors;
boost::array<int, 1> viewports;

bool onlyOne = true;
bool was[] = {false,false};
int tick = 0;

class SimpleKinectViewer {
public:

	SimpleKinectViewer() {
	}

	static void renderData(pcl::visualization::PCLVisualizer& viewer) {
		if(!was[0] || !was[1]){
			for(int i=0; i<DEVICE_NUMBER; ++i){
				int prev = (i + DEVICE_NUMBER - 1) % DEVICE_NUMBER;
				int next = (i + 1) % DEVICE_NUMBER;

				while(sensors[prev]->setLaser(false) != S_OK);
				cout << "set off " << prev << '\n';

				sensors[i]->getNextColorPointCloud();
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = sensors[i]->getNextColorPointCloud();

				while(sensors[next]->setLaser(true) != S_OK);
				cout << "set on " << next << '\n';

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				Eigen::Affine3f aux(Eigen::Affine3f::Identity());
				aux.translate(Eigen::Vector3f(2*i - 1,0,0));

				pcl::transformPointCloud(*cloud.get(), *newCloud.get(), aux);
				cloud = newCloud;

				cout << cloud->points.size() << '\n';
				if(cloud->points.size() > 0){
					std::string name = (boost::format("sample cloud %1%") % i).str();
					pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
					if (!viewer.updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, name)) {
						viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name, viewports[0]);
					} else {
						if(onlyOne && !was[i] && cloud->points.size() > 0){
							was[i]=true;
							std::string name = (boost::format("kinect%1%.pcd") % i).str();
							pcl::io::savePCDFileBinary(name , *cloud.get());
						}
					}
				}
			}
		}
	}

	static void ini(pcl::visualization::PCLVisualizer& viewer) {
		double split = 1.0 / viewports.size();
		for (int i = 0; i < viewports.size(); ++i) {
			int v(0);
			viewer.createViewPort(split*i, 0.0, split * (i + 1), 1.0, v);
			viewer.setBackgroundColor(0, 0.4, 20, v);
			viewer.addCoordinateSystem(1, v);
			viewports[i] = v;
			viewer.initCameraParameters();
			viewer.setCameraClipDistances(-1000000.001,1000000,v);
		}
		viewer.initCameraParameters();
		
		for(int i=0; i<DEVICE_NUMBER; ++i){
			sensors[i]->setNearMode(true);
		}
		for(int i=1; i<DEVICE_NUMBER; ++i){
			sensors[i]->setLaser(false);
		}
	}

	void run() {
		pcl::visualization::CloudViewer viewer("PCL OpenNI Viewer1");
		viewer.runOnVisualizationThreadOnce(ini);
		viewer.runOnVisualizationThread(renderData);

		while (!viewer.wasStopped()) {
			sleep(1);
		}
	}
};

int main() {
	sensors =  KinectSensor::getKinects(DEVICE_NUMBER);
	
	SimpleKinectViewer v;
	v.run();

	return 0;
}

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (500)
    {
      // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

FeatureCloud::PointCloud::Ptr createTemplate()
{
	FeatureCloud::PointCloud::Ptr temp(new FeatureCloud::PointCloud());

	const float depth = 0.25f, width = 0.25f, height=0.105f, backHeight = 0.23f;
	const float stepSize = 0.005f;

	//left
	for(float z=0; z <= depth; z += stepSize)
	{
		for(float y=0; y < height; y += stepSize)
		{
			pcl::PointXYZ result(width, y, z);
			temp->points.push_back(result);
		}
	}

	//front
	for(float x=0; x <= width; x += stepSize)
	{
		for(float y=0; y < height; y += stepSize)
		{
			pcl::PointXYZ result(x, y, 0);
			temp->points.push_back(result);
		}
	}

	//right
	for(float z=0; z <= depth; z += stepSize)
	{
		for(float y=0; y <= height; y += stepSize)
		{
			pcl::PointXYZ result(0, y, z);
			temp->points.push_back(result);
		}
	}

	//top
	for(float z=0; z <= depth; z += stepSize)
	{
		for(float x=0; x <= width; x += stepSize)
		{
			pcl::PointXYZ result(x, height, z);
			temp->points.push_back(result);
		}
	}

	//back
	for(float x=0; x < width; x += stepSize)
	{
		for(float y=0; y < backHeight; y += stepSize)
		{
			pcl::PointXYZ result(x, y, depth);
			temp->points.push_back(result);
		}
	}

	return temp;
}

// Align a collection of object templates to a sample point cloud
int
main23 (int argc, char **argv)
{

  // Load the object templates specified in the object_templates.txt file
  std::vector<FeatureCloud> object_templates;

	FeatureCloud template_cloud;
    template_cloud.setInputCloud(createTemplate());

  // Load the target cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile ("test1.pcd", *cloud);

  // Preprocess the cloud by...
  // ...removing distant points
  /*const float depth_limit = 1.0;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, depth_limit);
  pass.filter (*cloud);*/

  // ... and downsampling the point cloud
  const float voxel_grid_size = 0.01f;
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setInputCloud (cloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
  vox_grid.filter (*tempCloud);
  cloud = tempCloud;

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (cloud);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  template_align.align(template_cloud, best_alignment);
  //const FeatureCloud &best_template = object_templates[best_index];

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
  Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

  // Save the aligned template for visualization
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud (*template_cloud.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
  pcl::io::savePCDFileBinary ("output.pcd", transformed_cloud);
	
		//pcl::visualization::CloudViewer viewer("PCL OpenNI Viewer1");
		//
		//{
		//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		//	pcl::io::loadPCDFile ("test1.pcd", *cloud);
		//	viewer.showCloud(cloud);
		//}
		///*{
		//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		//	pcl::io::loadPCDFile ("output.pcd", *cloud);
		//	viewer.showCloud(cloud);
		//}*/

		//while(!viewer.wasStopped()){}

  return (0);
}


void downsample( pcl::PointCloud<pcl::PointXYZ>::Ptr points, float leafsize, pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_out)
{
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setInputCloud(points);
  vox_grid.setLeafSize (leafsize, leafsize, leafsize);
  vox_grid.filter(*downsampled_out);
}

void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals, float normal_radius)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setInputCloud (points);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method_xyz (new pcl::search::KdTree<pcl::PointXYZ>);
	norm_est.setSearchMethod (search_method_xyz);
	norm_est.setRadiusSearch (normal_radius);
	norm_est.compute (*normals);
}

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::PointXYZ>::Ptr scaled, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
	pcl::visualization::PCLVisualizer viz;
	viz.addPointCloud(points, "cloud3");
	viz.addPointCloud(scaled, "scaled4");
	viz.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(scaled, normals, 1, 0.);
	viz.spin();
}

void detectkeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, float minscale, int nroctaves, int nrscalesperoctave, float mincontrast, pcl::PointCloud<pcl::PointWithScale>::Ptr keypointsout)
{
	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> siftdetect;
	// Use a FLANNb a se d KdTree t o p e r f o rm n e i g h b o r h o o d s e a r c h e s
	siftdetect.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	// Se t t h e d e t e c t i o n p a r am e t e r s
	siftdetect.setScales(minscale, nroctaves, nrscalesperoctave) ;
	siftdetect.setMinimumContrast(mincontrast) ;
	// Se t t h e i n p u t
	siftdetect.setInputCloud(points);

	siftdetect.compute(*keypointsout);
}

int main22(int argc, char **argv){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile ("test0.pcd", *cloud1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile ("test1.pcd", *cloud2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr ds1 (new pcl::PointCloud<pcl::PointXYZ>); 
	downsample(cloud1, 0.01f, ds1);
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr ds2 (new pcl::PointCloud<pcl::PointXYZ>); 
	downsample(cloud2, 0.01f, ds2);*/

	
	/*pcl::PointCloud<pcl::PointWithScale>::Ptr keys (new pcl::PointCloud<pcl::PointWithScale>); 

	float a = 0.001f;
	detectkeypoints(ds1, a, 4, 5, 1.0f, keys);*/

	return 0;
}


