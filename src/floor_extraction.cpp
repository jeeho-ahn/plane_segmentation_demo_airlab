/*
Extract and find floor plane with similar orientation to a given reference vector using RANSAC and get it's normal vector
2021.3.2
Written by Ahn, Jeeho
MIT license
*/

const float dot_thres = 0.97;
bool save_pcd = false;

#include "floor_extraction.h"
pclCloud::Ptr uniform_filter(pclCloud::Ptr cloud_in)
{
    pclCloud::Ptr cloud_filtered(new pclCloud);
    pcl::UniformSampling<pclType> filter;
    filter.setInputCloud (cloud_in);
    filter.setRadiusSearch (0.015);
    filter.filter (*cloud_filtered);

    return cloud_filtered;
}

template<typename T>
std::shared_ptr<T> load_pcd_file(std::string fname)
{
    std::cout << "Loading PointCloud File: " << fname << std::endl;
    std::shared_ptr<T> out_cloud(new pclCloud);
    std::string last_three = fname.substr(fname.size() - 3, 3);

    if (last_three == "pcd")
    {
        if (pcl::io::loadPCDFile(fname, *out_cloud) == -1)
        {
            PCL_ERROR(std::string("Failed to load PCD File " + fname + "\n").c_str());
            exit(-2);
        }
        return out_cloud;
    }
    else if (last_three == "ply")
    {
        if (pcl::io::loadPLYFile(fname, *out_cloud) == -1)
        {
            PCL_ERROR(std::string("Failed to load PLY File " + fname + "\n").c_str());
            exit(-2);
        }
        return out_cloud;
    }
    else
    {
        PCL_ERROR(("Unknown extension: " + last_three).c_str());
        exit(-2);
    }
}

pcl::SACSegmentation<pclType> init_plane_seg(double distance_thres)
{
    // Create the segmentation object
    pcl::SACSegmentation<pclType> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(50);
    //seg.setNumberOfThreads(0);
    seg.setDistanceThreshold(distance_thres);

    return seg;
}

segmented_set segment_single_plane(pclCloud::Ptr cloud, double dist_thres)
{
    auto seg = init_plane_seg(dist_thres); //0.01
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return (segmented_set()); //filled with nullptr
    }

    /*
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
        << coefficients->values[1] << " "
        << coefficients->values[2] << " "
        << coefficients->values[3] << std::endl;
        */

    pclCloud::Ptr out_cloud_inlier(new pclCloud);
    pclCloud::Ptr out_cloud_outlier(new pclCloud);
    // Create the filtering object
    pcl::ExtractIndices<pclType> extract;
    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*out_cloud_inlier);

    extract.setNegative(true);
    extract.filter(*out_cloud_outlier);

    return segmented_set(out_cloud_inlier, out_cloud_outlier, coefficients);

}

std::pair<pclCloud::Ptr, plane_ptr> extract_floor(pclCloud::Ptr cloud_in, bool uniform_sample)
{
    std::vector<segmented_set> segmented_list;

    pclCloud::Ptr cloud(new pclCloud);
    if(uniform_sample)
      cloud = uniform_filter(cloud_in);
    else
      cloud = cloud_in;


    bool finished = false;
    pclCloud::Ptr pivot_cloud = cloud;

    //set max. num. of points left to repeat
    auto size_thres = cloud->size() / 4;

    auto time_start = std::chrono::system_clock::now();
    while (finished == false)
    {
        auto segmented_clouds = segment_single_plane(pivot_cloud, 0.01);
        if (segmented_clouds.outliers->size() < size_thres)
        {
            //std::cout << "Cloud Size thres reached. Terminate RANSAC" << std::endl;
            //save last plane to list
            segmented_list.push_back(segmented_clouds);
            //stop loop
            break;
        }
        if (segmented_clouds.inliers == nullptr)
        {
            std::cout << "No more Plane found" << std::endl;
            finished = true;
        }
        else
        {
            //save inlier to list
            segmented_list.push_back(segmented_clouds);
            //reiterate with leftover
            pivot_cloud = segmented_clouds.outliers;
        }
    }

    //std::cout << "Time Counted: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - time_start).count() << std::endl;
    //auto ref_vec = Eigen::Vector3f(0, 0.866, 0.5);
    auto ref_vec = EV3(0, 0, 1);
    std::vector<segmented_set> floor_angle_planes;
    if (segmented_list.size() == 0)
    {
        std::cout << "No Plane Found from the source cloud" << std::endl;
        return std::make_pair<pclCloud::Ptr,plane_ptr>(nullptr,nullptr);
    }
    else
    {
        std::cout << "\nPlanes Found: " << segmented_list.size() << std::endl;
        for (int n = 0; n < segmented_list.size(); n++)
        {
            if(save_pcd)
                pcl::io::savePCDFile("plane" + std::to_string(n) + ".pcd", *segmented_list[n].inliers);
            //std::cout << "Surface Normal: " << *(segmented_list[n].surf_normal()) << std::endl;
            float dot = ref_vec.dot(*(segmented_list[n].surf_normal()));
            //std::cout << "dot: " << dot << std::endl;
            if (fabs(dot) > dot_thres)
                floor_angle_planes.push_back(segmented_list[n]);
        }

        if (floor_angle_planes.size() == 0)
        {
            std::cout << "No Floor Plane Found from the source cloud" << std::endl;
            return std::make_pair<pclCloud::Ptr,plane_ptr>(nullptr, nullptr);
        }
        else if (floor_angle_planes.size() == 1)
        {
            std::cout << "Unique Floor normal:\n" << *floor_angle_planes[0].surf_normal() << std::endl;
            plane_class<double> plane_(floor_angle_planes[0].surf_coeff);

            return std::make_pair(floor_angle_planes[0].inliers, std::make_shared<plane_class<double>>(plane_));
        }
        else //multiple planes with similar angles found
        {
            //choose one with most number of points
            auto pivot = floor_angle_planes[0];
            for (int n = 1; n < floor_angle_planes.size(); n++)
            {
                if (floor_angle_planes[n].inliers->size() > pivot.inliers->size())
                    pivot = floor_angle_planes[n];
            }

            std::cout << "Best Floor normal:\n" << *pivot.surf_normal() << std::endl;
            plane_class<double> plane_(pivot.surf_coeff);
            return std::make_pair(pivot.inliers, std::make_shared<plane_class<double>>(plane_));
        }
    }
}

std::pair<pclCloud::Ptr, plane_ptr> extract_floor(pcl::PCLPointCloud2::Ptr cloud2_in, bool uniform_sample)
{
  pclCloud::Ptr cloud_in(new pclCloud);
  pcl::fromPCLPointCloud2(*cloud2_in,*cloud_in);
  return extract_floor(cloud_in, uniform_sample);
}
