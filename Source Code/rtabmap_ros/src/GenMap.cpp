#include "ros/ros.h"
#include "rtabmap_ros/MapData.h"
#include "rtabmap/core/Rtabmap.h"
#include <rtabmap/core/Transform.h>
#include "rtabmap_ros/MsgConversion.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_transforms.h"
#include "pcl_ros/point_cloud.h"
#include <rtabmap/utilite/UStl.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "sensor_msgs/PointCloud2.h"

using namespace rtabmap;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
ros::Publisher pub;
std::map<int, rtabmap::Signature> nodes;
pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void callback(const rtabmap_ros::MapDataConstPtr & msg){
	
	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> links;
	rtabmap::Transform mapOdom;

	for(unsigned int i=0; i<msg->nodes.size(); ++i){
		uInsert(::nodes, std::make_pair(msg->nodes[i].id, rtabmap_ros::nodeDataFromROS(msg->nodes[i])));
	}

	rtabmap_ros::mapGraphFromROS(msg->graph, poses, links, mapOdom);

	if(poses.size() && nodes.find(poses.rbegin()->first) != ::nodes.end()){
			rtabmap::Signature tmpS = nodes.at(poses.rbegin()->first);
			rtabmap::SensorData tmpData = tmpS.sensorData();
			tmpData.setId(0);

			uInsert(nodes, std::make_pair(0, Signature(0, -1, 0, tmpS.getStamp(), "", tmpS.getPose(), Transform(), tmpData)));
			poses.insert(std::make_pair(0, poses.rbegin()->second));
	}
	// for(unsigned int i=0; i<msg->graph.posesId.size(); ++i)
	// {
	// 	poses.insert(std::make_pair(msg->graph.posesId[i], rtabmap_ros::transformFromPoseMsg(msg->graph.poses[i])));
	// }
	// for(unsigned int i=0; i<msg->graph.links.size(); ++i)
	// {
	// 	//Transform t = rtabmap_ros::transformFromGeometryMsg(msg->graph.links[i].transform);
	// 	links.insert(std::make_pair(msg->graph.links[i].fromId, rtabmap_ros::linkFromROS(msg->graph.links[i])));
	// }

	for(std::map<int, rtabmap::Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		rtabmap::Signature node = nodes.find(iter->first)->second;

		// uncompress data
		node.sensorData().uncompressData();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = rtabmap::util3d::cloudRGBFromSensorData(
				node.sensorData(),
				4,           // image decimation before creating the clouds
				4.0f,        // maximum depth of the cloud
				0.0f);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::vector<int> index;
		//pcl::demeanPointCloud(*tmp, *tmpNoNaN, index);
		if(!tmpNoNaN->empty())
		{

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp = rtabmap::util3d::transformPointCloud(tmp, iter->second);
			*::cloud += *temp;
		 //transform the point cloud to its pose
		}
	}
	if(cloud->size())
	{
		sensor_msgs::PointCloud2 msg;
		printf("Voxel grid filtering of the assembled cloud (voxel=%f, %d points)\n", 0.01f, (int)cloud->size());
		cloud = rtabmap::util3d::voxelize(cloud, 0.01f);
		rgbVis(cloud);
		//printf("Saving rtabmap_cloud.pcd... done! (%d points)\n", (int)cloud->size());
		//pcl::io::savePCDFile("rtabmap_cloud.pcd", *cloud);
		//pcl::io::savePLYFile("rtabmap_cloud.ply", *cloud); // to save in PLY format
	}
	else
	{
		printf("Saving rtabmap_cloud.pcd... failed! The cloud is empty.\n");
	}
}



int main(int argc, char **argv){
	ros::init(argc, argv, "Gen_map");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<rtabmap_ros::MapData>("/rtabmap/mapData",1, callback);
	//pub = n.advertise<sensor_msgs::PointCloud2> ("pointcloud", 1);
	ros::spin();
}