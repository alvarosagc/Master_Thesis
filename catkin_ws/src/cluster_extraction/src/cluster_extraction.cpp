#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//Esto de aqui esta en otros tutoriales
int main () {
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("radius_outlier_pcd.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);
 
   int nr_points = (int) cloud_filtered->size ();
   while (cloud_filtered->size () > 1 * nr_points)
   {
     // Segment the largest planar component from the remaining cloud
     seg.setInputCloud (cloud_filtered);
     seg.segment (*inliers, *coefficients);
     if (inliers->indices.size () == 0)
     {
       std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
       break;
     }
 
     // Extract the planar inliers from the input cloud
     pcl::ExtractIndices<pcl::PointXYZ> extract;
     extract.setInputCloud (cloud_filtered);
     extract.setIndices (inliers);
     extract.setNegative (false);
 
     // Get the points associated with the planar surface
     extract.filter (*cloud_plane);
     std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl; 
 
     // Remove the planar inliers, extract the rest -- Me esta quitando los elementos que considera planares
     extract.setNegative (true);
     extract.filter (*cloud_f);
     *cloud_filtered = *cloud_f;
   }
 
   // Creating the KdTree object for the search method of the extraction -- Aqui se esta creando el Kd-tree que basicamente es el metodo de search que se utiliza en el algoritmo de extraccion
   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
   tree->setInputCloud (cloud_filtered);
 
 //El vector PointIndices incluye la informacion actual de los indices. Los indices para cada uno de los clusters detectados se almacena aqui. IMP este vector incluye una instancia para cada uno de los clusters detectados. por ejemplo: cluster_indices[0] contiene todos los indices para el primer cluster detectado en la point cloud
   std::vector<pcl::PointIndices> cluster_indices;
   //Se esta creando el objeto para la extraccion de cluster para la nube de tipo XYZ --  Coincide con el tipo de nube del proyecto
   pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
   
   //CUIDADADOOOO!!!!!  si se coge un valor muy pequenyo lo que puede ocurrir es que un mismo objeto se divida en distintos clusters, tambien puede ocurrir al reves. hay que ir probando hasta que se obtiene lo que quiere
   ec.setClusterTolerance (0.07); // cm
   
   //Definicion del numero minimo y maximo de puntos que un cluster puede tener
   ec.setMinClusterSize (60);
   ec.setMaxClusterSize (10000);
   ec.setSearchMethod (tree);
   ec.setInputCloud (cloud_filtered);
   //Aqui es donde se guardan los indices de los objetos que se han extraido 
   ec.extract (cluster_indices);
 
 //En este bucle lo que se esta es iterando en el vector point_indices para separar cada uno de los clusters en una nube de puntos separada 
   int j = 0;
   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
   {
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
     for (const auto& idx : it->indices)
     cloud_cluster->push_back ((*cloud_filtered)[idx]); //*
     cloud_cluster->width = cloud_cluster->size ();
     cloud_cluster->height = 1;
     cloud_cluster->is_dense = true;
 
     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
     std::stringstream ss;
     ss << "cloud_cluster_" << j << ".pcd";
     writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
     j++;
   }

  return (0);
}