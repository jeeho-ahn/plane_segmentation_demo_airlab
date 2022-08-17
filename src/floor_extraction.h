#ifndef FLOOR_EXTRACTION_H
#define FLOOR_EXTRACTION_H

#include <iostream>
//pcl
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/conversions.h>

#include <Eigen/Dense>

#include <memory>
#include <chrono>

typedef pcl::PointXYZ pclType;
typedef pcl::PointCloud<pclType> pclCloud;

typedef Eigen::Vector3d EV3;
typedef std::shared_ptr<EV3> EV3_ptr;

extern const float dot_thres;
extern bool save_pcd;

template<typename precision>
class plane_class
{
    typedef Eigen::Matrix<precision, 3, 1> vec3;
    public:
    vec3 normal;
    precision d; //4th coefficient in std eq.

    plane_class()
    {
        normal = vec3(0,0,0);
        d = 0;
    }

    plane_class(pcl::ModelCoefficients::Ptr mc)
    {
        normal = vec3((precision)mc->values[0],
                      (precision)mc->values[1],
                      (precision)mc->values[2]);
        d = (precision)mc->values[3];
    }
};

typedef std::shared_ptr<plane_class<double>> plane_ptr;


class segmented_set
{
public:
    pclCloud::Ptr inliers;
    pclCloud::Ptr outliers;
    pcl::ModelCoefficients::Ptr surf_coeff;

    segmented_set()
    {
        inliers = nullptr;
        outliers = nullptr;
        surf_coeff = nullptr;
    }

    segmented_set(pclCloud::Ptr inl, pclCloud::Ptr outl, pcl::ModelCoefficients::Ptr coeff)
    {
        inliers = inl;
        outliers = outl;
        surf_coeff = coeff;
    }
    EV3_ptr surf_normal()
    {
        return std::make_shared<EV3>(EV3(surf_coeff->values[0], surf_coeff->values[1], surf_coeff->values[2]));
    }
};

pclCloud::Ptr uniform_filter(pclCloud::Ptr cloud_in);

template<typename T>
std::shared_ptr<T> load_pcd_file(std::string fname);

pcl::SACSegmentation<pclType> init_plane_seg(double distance_thres);

segmented_set segment_single_plane(pclCloud::Ptr cloud, double dist_thres);
std::pair<pclCloud::Ptr, plane_ptr> extract_floor(pclCloud::Ptr cloud, bool uniform_sample);
std::pair<pclCloud::Ptr, plane_ptr> extract_floor(pcl::PCLPointCloud2::Ptr cloud2_in, bool uniform_sample);


#endif // FLOOR_EXTRACTION_H
