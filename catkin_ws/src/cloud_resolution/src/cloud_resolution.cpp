#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>
 
int main () {
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  
  reader.read ("scene_filename_2.pcd", *cloud);
  pcl::search::KdTree<pcl::PointXYZ>  tree;
  tree.setInputCloud (cloud);
   
  for (std::size_t i = 0; i < cloud->size (); ++i){
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  
    std::cout << "Model resolution: " << res<< std::endl;
	
  return 0;
}