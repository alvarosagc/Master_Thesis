 #include <pcl/io/pcd_io.h>
 #include <ros/ros.h>
 #include <pcl/point_cloud.h>
 #include <pcl_conversions/pcl_conversions.h>
 #include <sensor_msgs/PointCloud2.h>
 #include <pcl/correspondence.h>
 #include <pcl/features/normal_3d_omp.h>
 #include <pcl/features/shot_omp.h>
 #include <pcl/features/board.h>
 #include <pcl/filters/uniform_sampling.h>
 #include <pcl/recognition/cg/hough_3d.h>
 #include <pcl/recognition/cg/geometric_consistency.h>
 #include <pcl/visualization/pcl_visualizer.h>
 #include <pcl/kdtree/kdtree_flann.h>
 #include <pcl/kdtree/impl/kdtree_flann.hpp>
 #include <pcl/common/transforms.h>
 #include <pcl/console/parse.h>
 #include <pcl/keypoints/sift_keypoint.h>
 #include <pcl/keypoints/iss_3d.h>
 #include <pcl/filters/radius_outlier_removal.h>
 #include <pcl/filters/voxel_grid.h>
 
//typedef declaration of objects to save time in the declarations in the program
 typedef pcl::PointXYZ PointType;
 typedef pcl::Normal NormalType;
 typedef pcl::ReferenceFrame RFType;
 typedef pcl::SHOT352 DescriptorType;
 
//Model filename name, for charging it later from the terminal
 std::string model_filename_;
//Scene filename name, for charging it later from the terminal
 std::string scene_filename_;
 
// Definition of parameters for the algorithms implicated in the process
 bool show_keypoints_ (false);
 bool show_correspondences_ (false);
 bool use_cloud_resolution_ (false);
 bool use_hough_ (true);

// Parameters Uniform Sampling Keypoint extraction

 float model_ss_ (0.02f); //keypoint sampling dimensions model
 float scene_ss_ (0.03f); //keypoint sampling dimensions scene
 
 //Parameters SIFT Keypoint extraction
 
 //Model
 float min_scale_mod (0.01f); //Standard deviation of the smallest scale
 int number_oct_mod (10); //Number of groups in the Gaussian pyramid
 int number_scales_octave_mod (16); //Number of scales per group
 float min_contrast_mod (0.00001f); //Threshold for Keypoint detection
 
 //Scene
 float min_scale_sce (0.01f); //Standard deviation of the smallest scale
 int number_oct_sce (18); //Number of groups in the Gaussian pyramid
 int number_scales_octave_sce (24); //Number of scales per group
 float min_contrast_sce (0.00001f); //Threshold for Keypoint detection
 
 // Parameters ISS Keypoint extraction
 double cloud_resolution_mod (0.0226285);
 float set_Threshold21_mod (0.975f);
 float set_Threshold32_mod (0.975f);
 int min_neigh_mod (2);
 int num_threads_mod (1);
 
 double cloud_resolution_sce (0.0126568);
 float set_Threshold21_sce (0.975f);
 float set_Threshold32_sce (0.975f);
 int min_neigh_sce (2);
 int num_threads_sce (1);
 
 
 //Recognition and Registration parameters
 float rf_rad_ (0.035f); // Hough3D algorithm
 float descr_rad_ (0.15f); //features SHOT extraction
 float cg_size_ (0.01f); //Size of the Hough3D subdivisions
 float cg_thresh_ (5.0f); //In the researcher document the
 
 
 //void function for files obtention from the terminal

 void parseCommandLine (int argc, char *argv[]) {
   
   //Model & scene filenames, vector for taking the names from the terminal -- More flexibility for testing
   std::vector<int> docs;
   
  //Termination (.pcd) required for the files of the input
   docs = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
   
   //If you don enter 2 files show error and quit the function
   if (docs.size () != 2) {
     std::cout << "Filenames missing.\n";
     exit (-1);
   }
   
   //Storing of the filenames intp variables 
   model_filename_ = argv[docs[0]];
   scene_filename_ = argv[docs[1]];
  
}

//For creation of SIFT Keypoints

//The SIFT keypoints need to have a value of the intensity, in this case the depth will be used as the intensity
namespace pcl {
  template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
      inline float
      operator () (const PointXYZ &p) const
      {
	return p.x;
      }
    };
}


//Starting point for the registration // Recognition

int main (int argc, char *argv[]) {
	
  parseCommandLine (argc, argv);
  
   //Declaration of variables needed for the algorithms below
   
  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_prepro (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());
  
  //PC load from files obtained in the first function -- The load is done in the condition field of the if statement, then a message is shown if the load does not work properly
  
   if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    return (-1);
  }
  
  //Final program, lecture from the initial cloud topic
  //sub = nh.subscribe("initial_PC2", 10, *scene);
  
  if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    return (-1);
  }
  
  //Outlier removal
  
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_Filter;
  
  radius_Filter.setInputCloud(scene);
  radius_Filter.setRadiusSearch(0.1);
  radius_Filter.setMinNeighborsInRadius (6);
  radius_Filter.setKeepOrganized(false);
  
  radius_Filter.filter (*scene_prepro);
  
  //Downsampling - Use only in the case where uniform_sampling is not the method for keypoint extraction because is based in the same concept and will cause to do a double effort
  
  	pcl::VoxelGrid<pcl::PointXYZ> voxel_downsampling;
			
	voxel_downsampling.setInputCloud (scene_prepro);
	voxel_downsampling.setLeafSize(0.01f, 0.01f, 0.01f);
	voxel_downsampling.filter (*scene_prepro);
  
  
  
  //Computation of the normals for each of the points
  
  //Creation of the Normal estimation object
  pcl::NormalEstimationOMP<PointType, NormalType> normal_est;
  
  normal_est.setKSearch (20); //Number of points used for the Kd-Tree search
  
  //Model
  normal_est.setInputCloud (model);
  normal_est.compute (*model_normals);
  
  //Scene
  normal_est.setInputCloud (scene_prepro);
  normal_est.compute (*scene_normals);
  
  
  //Keypoints extraction
  
  //Creacion del objeto para hacer el keypoints extraction
  pcl::UniformSampling<PointType> keypoints_uniform;
  
  //Keypoints extraction for the model
  
  //UNIFORM SAMPLING
  keypoints_uniform.setInputCloud (model);
  keypoints_uniform.setRadiusSearch (model_ss_);
  keypoints_uniform.filter (*model_keypoints);
  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  //Keypoints extraction for the scene
  keypoints_uniform.setInputCloud (scene_prepro);
  keypoints_uniform.setRadiusSearch (scene_ss_);
  keypoints_uniform.filter (*scene_keypoints);
  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;
  
  // SIFT KEYPOINT EXTRACTION
  /*
  pcl::SIFTKeypoint <PointType, pcl::PointWithScale> keypoints_SIFT;
  pcl::search::KdTree <PointType>::Ptr tree_SIFT (new pcl::search::KdTree<PointType>());
  
  pcl::PointCloud<pcl::PointWithScale> result_model;
  keypoints_SIFT.setInputCloud (model);
  keypoints_SIFT.setSearchMethod (tree_SIFT);
  keypoints_SIFT.setScales (min_scale_mod, number_oct_mod, number_scales_octave_mod);
  keypoints_SIFT.setMinimumContrast (min_contrast_mod);
  keypoints_SIFT.compute (result_model);
  copyPointCloud(result_model, *model_keypoints);
  
  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;
  
  pcl::PointCloud<pcl::PointWithScale> result_scene;
  keypoints_SIFT.setInputCloud (scene);
  keypoints_SIFT.setSearchMethod (tree_SIFT);
  keypoints_SIFT.setScales (min_scale_sce, number_oct_sce, number_scales_octave_sce);
  keypoints_SIFT.setMinimumContrast (min_contrast_sce);
  keypoints_SIFT.compute (result_scene);
  copyPointCloud(result_scene, *scene_keypoints);
  std::cout << "Model total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;*/
  
  // ISS KEYPOINT EXTRACTION
  
  /*pcl::ISSKeypoint3D <PointType, PointType>  keypoints_ISS;
  pcl::search::KdTree <PointType>::Ptr tree_ISS (new pcl::search::KdTree<PointType>());
  
  keypoints_ISS.setSearchMethod (tree_ISS);
  keypoints_ISS.setSalientRadius (1 * cloud_resolution_mod);
  keypoints_ISS.setNonMaxRadius (1 * cloud_resolution_mod);
  keypoints_ISS.setThreshold21 (set_Threshold21_mod);
  keypoints_ISS.setThreshold32 (set_Threshold32_mod);
  keypoints_ISS.setMinNeighbors (min_neigh_mod);
  keypoints_ISS.setNumberOfThreads (num_threads_mod);
  keypoints_ISS.setInputCloud (model);
  keypoints_ISS.compute (*model_keypoints);
  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;
  
  keypoints_ISS.setSearchMethod (tree_ISS);
  keypoints_ISS.setSalientRadius (1 * cloud_resolution_sce);
  keypoints_ISS.setNonMaxRadius (2 * cloud_resolution_sce);
  keypoints_ISS.setThreshold21 (set_Threshold21_sce);
  keypoints_ISS.setThreshold32 (set_Threshold32_sce);
  keypoints_ISS.setMinNeighbors (min_neigh_sce);
  keypoints_ISS.setNumberOfThreads (num_threads_sce);
  keypoints_ISS.setInputCloud (scene);
  keypoints_ISS.compute (*scene_keypoints);
  std::cout << "Model total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;*/
  
//Features extraction
  
  //The method used is the SHOTEstimator, based in the OMP, the same type of the normal estimator  
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> SHOT_descr;
  
  //Set of parameter -- Remember that all the parameters are defined at the begining
  SHOT_descr.setRadiusSearch (descr_rad_);

//Feature extraction for the model
  SHOT_descr.setInputCloud (model_keypoints);
  SHOT_descr.setInputNormals (model_normals);
  SHOT_descr.setSearchSurface (model);
  SHOT_descr.compute (*model_descriptors);

//Feature extraction for the scene
  SHOT_descr.setInputCloud (scene_keypoints);
  SHOT_descr.setInputNormals (scene_normals);
  SHOT_descr.setSearchSurface (scene_prepro);
  SHOT_descr.compute (*scene_descriptors);
  

//Registration -- Obtention of correspondences
  
  //Correspondences Object Creation
  pcl::CorrespondencesPtr correspondence_array (new pcl::Correspondences ());

//Creation of the Kd-Tree for NN search
  pcl::KdTreeFLANN<DescriptorType> NN_search;
  
  NN_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  
  for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = NN_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)  //Correspondences rejection based in the SHOT descriptors
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      correspondence_array->push_back (corr);
    }
  }
  // Information about the number of correspondences -- IMP for iteration of algorithms
  std::cout << "Correspondences found: " << correspondence_array->size () << std::endl;
  
  
  //Actual clustering -- Creo que sera sacar la transformada homogenea
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;
  
  //Aqui se esta utilizando hough para ver si se encuentra alguna coincidencia entre modelo y lo que se tiene en la figura general 
  if (use_hough_)
  {
    //
    //  Compute (Keypoints) Reference Frames only for Hough -- Recuerda ponerlo aqui poque solo es necesario si se quiere extraer siguiendo el algoritmo de Hough 
    // Recuerda que el algoritmo de Hough (Investigar mas de esto si este algoritmo funciona) asocia una LRF (local reference frame) a cada uno de los keypoints pertenecientes a las nubes MIRAR LOS TUTORIALS DEL PCL
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene_prepro);
    rf_est.compute (*scene_rf);

    //  Clustering -- Despues de sacar los LRF es cuando se llama al algoritmo de Hough 
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (correspondence_array);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
  }
  /* METHOD B IN CASE THE FIRST ONE DOES NOT WORK
  else // Using GeometricConsistency -- aplicacion del otro algoritmo que se tiene 
  {'
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }*/
  
  
    //  Output results
	
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }
  
  //  Visualization
  //
  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  viewer.addPointCloud (scene, "scene_cloud");

  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

 /* if (show_correspondences_ || show_keypoints_)
  {
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }

  if (show_keypoints_)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }*/

  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
  viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());
  }

   /* if (show_correspondences_)
    {
      for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }
  }*/

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);
}