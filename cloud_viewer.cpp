
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/features/pfh.h>	
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

#define SCAN 0
#if SCAN == 1
const int DEVICE_NUMBER = 2;
std::vector<KinectSensor::Ptr> sensors;
boost::array<int, 1> viewports;

bool onlyOne = true;
bool was[DEVICE_NUMBER];
int tick = 0;

class SimpleKinectViewer {
public:

	SimpleKinectViewer() {
	}

	static void renderData(pcl::visualization::PCLVisualizer& viewer) {
		bool wasAll = true;
		for(int i=0; i<DEVICE_NUMBER; ++i){
			wasAll &= was[i];
		}

		if(!wasAll){
			for(int i=0; i<DEVICE_NUMBER; ++i){
				int prev = (i + DEVICE_NUMBER - 1) % DEVICE_NUMBER;
				int next = (i + 1) % DEVICE_NUMBER;

				if(DEVICE_NUMBER > 1){
					while(sensors[prev]->setLaser(false) != S_OK);
					cout << "set off " << prev << '\n';
				}

				sensors[i]->getNextColorPointCloud();
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = sensors[i]->getNextColorPointCloud();

				if(DEVICE_NUMBER > 1){
					while(sensors[next]->setLaser(true) != S_OK);
					cout << "set on " << next << '\n';
				}

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
		
		/*for(int i=0; i<DEVICE_NUMBER; ++i){
			sensors[i]->setNearMode(true);
		}*/
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
	for(int i=0; i<DEVICE_NUMBER; ++i){
		was[i] = false;
	}

	sensors =  KinectSensor::getKinects(DEVICE_NUMBER);
	for(int i=0; i< DEVICE_NUMBER; i++)
	{
		if(sensors[i].get() == NULL)
		{			
			printf ("ERROR: %d Kinects connected. %d Kinects requested!", i, DEVICE_NUMBER);
			return 1;
		}
	}
	
	SimpleKinectViewer v;
	v.run();

	return 0;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr createTemplate()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>());

	const float depth = 0.25f, width = 0.25f, height=0.105f, backHeight = 0.23f;
	const float stepSize = 0.005f;

	//left
	for(float z=0; z <= depth; z += stepSize)
	{
		for(float y=0; y < height; y += stepSize)
		{
			pcl::PointXYZRGB result(width, y, z);
			temp->points.push_back(result);
		}
	}

	//front
	for(float x=0; x <= width; x += stepSize)
	{
		for(float y=0; y < height; y += stepSize)
		{
			pcl::PointXYZRGB result(x, y, 0);
			temp->points.push_back(result);
		}
	}

	//right
	for(float z=0; z <= depth; z += stepSize)
	{
		for(float y=0; y <= height; y += stepSize)
		{
			pcl::PointXYZRGB result(0, y, z);
			temp->points.push_back(result);
		}
	}

	//top
	for(float z=0; z <= depth; z += stepSize)
	{
		for(float x=0; x <= width; x += stepSize)
		{
			pcl::PointXYZRGB result(x, height, z);
			temp->points.push_back(result);
		}
	}

	//back
	for(float x=0; x < width; x += stepSize)
	{
		for(float y=0; y < backHeight; y += stepSize)
		{
			pcl::PointXYZRGB result(x, y, depth);
			temp->points.push_back(result);
		}
	}

	return temp;
}

#else

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(pcl::PointCloud<PointT>::Ptr &points, float normal_radius)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal > norm_est;
	norm_est.setInputCloud (points);
	pcl::search::KdTree<PointT>::Ptr search_method_xyz (new pcl::search::KdTree<PointT>);
	norm_est.setSearchMethod (search_method_xyz);
	norm_est.setRadiusSearch (normal_radius);
	norm_est.compute (*normals);
	return normals;
}

//template<typename PointT>
pcl::PointCloud<PointT>::Ptr downsample(pcl::PointCloud<PointT>::Ptr &points, float leafsize)
{
  pcl::PointCloud<PointT>::Ptr downsampled_out (new pcl::PointCloud<PointT>); 

  pcl::VoxelGrid<PointT> vox_grid;
  vox_grid.setInputCloud(points);
  vox_grid.setLeafSize (leafsize, leafsize, leafsize);
  vox_grid.filter(*downsampled_out);

  return downsampled_out;
}

LocalFeatures::Ptr	computeLocalFeatures (pcl::PointCloud<PointT>::Ptr &points, pcl::PointCloud<pcl::Normal>::Ptr &normals, float feature_radius)
{
	LocalFeatures::Ptr features_ = LocalFeatures::Ptr (new LocalFeatures);

	pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud (points);
	fpfh_est.setInputNormals (normals);
	fpfh_est.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
	fpfh_est.setRadiusSearch (feature_radius);
	fpfh_est.compute (*features_);

	return features_;
}

//template<typename PointT>
void visualize(pcl::PointCloud<PointT>::Ptr points, pcl::PointCloud<PointT>::Ptr scaled)
{
	pcl::visualization::PCLVisualizer viz;
	viz.addPointCloud(points, "cloud3");
	viz.addPointCloud(scaled, "scaled4");
	//viz.addPointCloudNormals<PointT, pcl::Normal>(scaled, normals, 1, 0.);
	viz.spin();
}

pcl::PointCloud<pcl::PointWithScale>::Ptr detectkeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float minscale, int nroctaves, int nrscalesperoctave, float mincontrast)
{
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypointsout (new pcl::PointCloud<pcl::PointWithScale>); 

	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> siftdetect;
	// Use a FLANNb a se d KdTree t o  p e r f o rm  n e i g h b o r h o o d  s e a r c h e s
	siftdetect.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	// Se t  t h e  d e t e c t i o n  p a r am e t e r s
	siftdetect.setScales(minscale, nroctaves, nrscalesperoctave) ;
	siftdetect.setMinimumContrast(mincontrast) ;
	// Se t  t h e  i n p u t
	siftdetect.setInputCloud(points);

	siftdetect.compute(*keypointsout);

	return keypointsout;
}

pcl::PointCloud<pcl::PFHSignature125>::Ptr
compute_PFH_features_at_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, 
                                   pcl::PointCloud<pcl::Normal>::Ptr &normals, 
                                   pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius)
{
  pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_out (new pcl::PointCloud<pcl::PFHSignature125>); 

  // Create a PFHEstimation object
  pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est;

  // Set it to use a FLANN-based KdTree to perform its neighborhood searches
  pfh_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));

  // Specify the radius of the PFH feature
  pfh_est.setRadiusSearch (feature_radius);

  /* This is a little bit messy: since our keypoint detection returns PointWithScale points, but we want to
   * use them as an input to our PFH estimation, which expects clouds of PointXYZRGB points.  To get around this,
   * we'll use copyPointCloud to convert "keypoints" (a cloud of type PointCloud<PointWithScale>) to 
   * "keypoints_xyzrgb" (a cloud of type PointCloud<PointXYZRGB>).  Note that the original cloud doesn't have any RGB 
   * values, so when we copy from PointWithScale to PointXYZRGB, the new r,g,b fields will all be zero.
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);

  // Use all of the points for analyzing the local structure of the cloud
  pfh_est.setSearchSurface (points);  
  pfh_est.setInputNormals (normals);  

  // But only compute features at the keypoints
  pfh_est.setInputCloud (keypoints_xyzrgb);

  // Compute the features
  pfh_est.compute (*descriptors_out);

  return descriptors_out;
}

void
find_feature_correspondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                              pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                              std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{
  // Resize the output vector
  correspondences_out.resize (source_descriptors->size ());
  correspondence_scores_out.resize (source_descriptors->size ());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source_descriptors->size (); ++i)
  {
    descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
    correspondences_out[i] = k_indices[0];
    correspondence_scores_out[i] = k_squared_distances[0];
  }
}

void visualize_keypoints (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
                          const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints)
{
  // Add the points to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");

  // Draw each keypoint as a sphere
  for (size_t i = 0; i < keypoints->size (); ++i)
  {
    // Get the point data
    const pcl::PointWithScale & p = keypoints->points[i];

    // Pick the radius of the sphere *
    float r = 2 * p.scale;
    // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
    //   radius of 2*p.scale is a good illustration of the extent of the keypoint

    // Generate a unique string for each sphere
    std::stringstream ss ("keypoint");
    ss << i;

    // Add a sphere at the keypoint
    viz.addSphere (p, 2*p.scale, 1.0, 0.0, 0.0, ss.str ());
  }

  // Give control over to the visualizer
  viz.spin ();
}

void visualize_correspondences (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1,
                                const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1,
                                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2,
                                const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2,
                                const std::vector<int> &correspondences,
                                const std::vector<float> &correspondence_scores)
{
  // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
  // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points

  // Create some new point clouds to hold our transformed data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_left (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_right (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointWithScale>);

  // Shift the first clouds' points to the left
  //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
  const Eigen::Vector3f translate (0.4, 0.0, 0.0);
  const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
  pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
  pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);

  // Shift the second clouds' points to the right
  pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
  pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);

  // Add the clouds to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points_left, "points_left");
  viz.addPointCloud (points_right, "points_right");

  // Compute the median correspondence score
  std::vector<float> temp (correspondence_scores);
  std::sort (temp.begin (), temp.end ());
  float median_score = temp[temp.size ()/2];

  // Draw lines between the best corresponding points
  for (size_t i = 0; i < keypoints_left->size (); ++i)
  {
    if (correspondence_scores[i] > median_score)
    {
      continue; // Don't draw weak correspondences
    }

    // Get the pair of points
    const pcl::PointWithScale & p_left = keypoints_left->points[i];
    const pcl::PointWithScale & p_right = keypoints_right->points[correspondences[i]];

    // Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max (r, std::max (g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;

    // Generate a unique string for each line
    std::stringstream ss ("line");
    ss << i;

    // Draw the line
    viz.addLine (p_left, p_right, r, g, b, ss.str ());
  }

  // Give control over to the visualizer
  viz.spin ();
}

void keypointDemo(const char* filename)
{	
  // Create some new point clouds to hold our data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Load a point cloud
  pcl::io::loadPCDFile (filename, *points);

  // Compute keypoints
  const float min_scale = 0.1;
  const int nr_octaves = 3;
  const int nr_octaves_per_scale = 3;
  const float min_contrast = 10.0;
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints = detectkeypoints (points, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast);

  // Visualize the point cloud and its keypoints
	visualize_keypoints (points, keypoints);
}

void correspondences_demo (const char * filename_base)
{
	// Load the pair of point clouds
	std::stringstream ss1, ss2;
	ss1 << filename_base << "1.pcd";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile (ss1.str (), *points1);

	ss2 << filename_base << "2.pcd";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile (ss2.str (), *points2);

	// Downsample the cloud
	const float voxel_grid_leaf_size = 0.01;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled1 = downsample (points1, voxel_grid_leaf_size);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled2 = downsample (points2, voxel_grid_leaf_size);

	// Compute surface normals
	const float normal_radius = 0.05;
	pcl::PointCloud<pcl::Normal>::Ptr normals1 = computeNormals(downsampled1, normal_radius);
	pcl::PointCloud<pcl::Normal>::Ptr normals2  = computeNormals(downsampled2, normal_radius);

	// Compute keypoints
	const float min_scale = 0.01;
	const int nr_octaves = 3;
	const int nr_octaves_per_scale = 3;
	const float min_contrast = 10.0;
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1 = detectkeypoints (points1, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2 =  detectkeypoints (points2, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast);

	// Compute PFH features
	const float feature_radius = 0.08;
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors1 = compute_PFH_features_at_keypoints (downsampled1, normals1, keypoints1, feature_radius);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors2 = compute_PFH_features_at_keypoints (downsampled2, normals2, keypoints2, feature_radius);

	// Find feature correspondences
	std::vector<int> correspondences;
	std::vector<float> correspondence_scores;
	find_feature_correspondences (descriptors1, descriptors2, correspondences, correspondence_scores);

	// Print out ( number of keypoints / number of points )
	std::cout << "First cloud: Found " << keypoints1->size () << " keypoints "
		<< "out of " << downsampled1->size () << " total points." << std::endl;
	std::cout << "Second cloud: Found " << keypoints2->size () << " keypoints "
		<< "out of " << downsampled2->size () << " total points." << std::endl;

//pcl::SampleConsensusInitialAlignment<pcl::PointWithScale, pcl::PointWithScale, pcl::PFHSignature125> sac_ia;
//sac_ia.setNumberOfSamples (25);
//sac_ia.setMinSampleDistance (0.005);
////sac_ia.setCorrespondenceRandomness (k);
//sac_ia.setMaximumIterations (300);
//sac_ia.setInputCloud (keypoints1);
//sac_ia.setInputTarget (keypoints2);
//sac_ia.setSourceFeatures (descriptors1);
//sac_ia.setTargetFeatures (descriptors2);
//
//pcl::PointCloud<pcl::PointWithScale>::Ptr aligned(new pcl::PointCloud<pcl::PointWithScale>());
//sac_ia.align (*aligned);
//Eigen::Matrix4f aaa = sac_ia.getFinalTransformation ();
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points11 (new pcl::PointCloud<pcl::PointXYZRGB>);
//				pcl::transformPointCloud(*points1.get(), *points11.get(), aaa);
//
//	// Visualize the two point clouds and their feature correspondences
//	visualize (points11, points2);

	visualize_correspondences(points1, keypoints1, points2, keypoints2, correspondences, correspondence_scores);
}

int main(int argc, char **argv){
	//keypointDemo("kinect1.pcd");
	//keypointDemo("robot2.pcd");
	correspondences_demo("robot");
  return (0);
}

#endif


