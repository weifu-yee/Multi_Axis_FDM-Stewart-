/*
 * TFTransform.cpp
 *
 *  Created on: Nov 13, 2024
 *      Author: user
 */

#include "TFTransform.h"
#include <Eigen/Geometry>

TFTransformer::TFTransformer()
    : word2nozzleTransform((Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ())).matrix()), // 旋轉和平移示例
      bed_surface2workpiece_originTransform((Eigen::Translation3d(0.0, 2.0, 0.0) * Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY())).matrix()),
      bed_joint_plane2bed_surfaceTransform((Eigen::Translation3d(0.0, 0.0, 3.0) * Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX())).matrix()) {

    // 初始化非 const 變換矩陣
    part2nozzleTransform = Eigen::Translation3d(1.0, 1.0, 0.0) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()); // 示例
    workpiece_origin2partTransform = Eigen::Translation3d(0.5, 0.5, 0.5) * Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()); // 示例
}

Eigen::Affine3d TFTransformer::getTransform(const std::string& from, const std::string& to) const {
    if (from == "WORLD_FRAME" && to == "NOZZLE_FRAME") {
        return word2nozzleTransform;
    } else if (from == "PART_FRAME" && to == "NOZZLE_FRAME") {
        return part2nozzleTransform;
    } else if (from == "WORKPIECE_ORIGIN_FRAME" && to == "PART_FRAME") {
        return workpiece_origin2partTransform;
    } else if (from == "BED_SURFACE_FRAME" && to == "WORKPIECE_ORIGIN_FRAME") {
        return bed_surface2workpiece_originTransform;
    } else if (from == "BED_JOINT_PLANE_FRAME" && to == "BED_SURFACE_FRAME") {
        return bed_joint_plane2bed_surfaceTransform;
    } else {
        return Eigen::Affine3d::Identity();
    }
}

Eigen::Affine3d TFTransformer::getJointPlanePoseInWorldFrame() const {
    // 取得從 WORLD_FRAME 到 BED_JOINT_PLANE_FRAME 的總變換
    return getTransform("WORLD_FRAME", "NOZZLE_FRAME") *
           getTransform("NOZZLE_FRAME", "PART_FRAME").inverse() *
           getTransform("PART_FRAME", "WORKPIECE_ORIGIN_FRAME").inverse() *
           getTransform("WORKPIECE_ORIGIN_FRAME", "BED_SURFACE_FRAME").inverse() *
           getTransform("BED_SURFACE_FRAME", "BED_JOINT_PLANE_FRAME").inverse();
}

