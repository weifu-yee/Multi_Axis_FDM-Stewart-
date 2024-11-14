/*
 * TFTransform.cpp
 *
 *  Created on: Nov 13, 2024
 *      Author: user
 */

#include "TFTransform.h"
#include <Eigen/Geometry>
#include "constants.h"

TFTransformer transformer;

// 將角度轉換為弧度的函數
double degreeToRad(double angle_in_degrees) {
    return angle_in_degrees * M_PI / 180.0;
}

TFTransformer::TFTransformer()
    : word2nozzleTransform(Eigen::Translation3d(WORD2NOZZLE_TRANSLATION_X, WORD2NOZZLE_TRANSLATION_Y, WORD2NOZZLE_TRANSLATION_Z) *
                           (Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(WORD2NOZZLE_ROTATION_X_DEGREE), Eigen::Vector3d::UnitX())) *
                            Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(WORD2NOZZLE_ROTATION_Y_DEGREE), Eigen::Vector3d::UnitY())) *
                            Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(WORD2NOZZLE_ROTATION_Z_DEGREE), Eigen::Vector3d::UnitZ())))),

      bed_surface2workpiece_originTransform(Eigen::Translation3d(BED_SURFACE2WORKPIECE_ORIGIN_TRANSLATION_X, BED_SURFACE2WORKPIECE_ORIGIN_TRANSLATION_Y, BED_SURFACE2WORKPIECE_ORIGIN_TRANSLATION_Z) *
                                            (Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(BED_SURFACE2WORKPIECE_ORIGIN_ROTATION_X_DEGREE), Eigen::Vector3d::UnitX())) *
                                             Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(BED_SURFACE2WORKPIECE_ORIGIN_ROTATION_Y_DEGREE), Eigen::Vector3d::UnitY())) *
                                             Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(BED_SURFACE2WORKPIECE_ORIGIN_ROTATION_Z_DEGREE), Eigen::Vector3d::UnitZ())))),

      bed_joint_plane2bed_surfaceTransform(Eigen::Translation3d(BED_JOINT_PLANE2BED_SURFACE_TRANSLATION_X, BED_JOINT_PLANE2BED_SURFACE_TRANSLATION_Y, BED_JOINT_PLANE2BED_SURFACE_TRANSLATION_Z) *
                                           (Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(BED_JOINT_PLANE2BED_SURFACE_ROTATION_X_DEGREE), Eigen::Vector3d::UnitX())) *
                                            Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(BED_JOINT_PLANE2BED_SURFACE_ROTATION_Y_DEGREE), Eigen::Vector3d::UnitY())) *
                                            Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(BED_JOINT_PLANE2BED_SURFACE_ROTATION_Z_DEGREE), Eigen::Vector3d::UnitZ())))) {
    // 初始化非 const 變換矩陣
    setPartToNozzleTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    setWorkpieceOriginToPartTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
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

SPPose TFTransformer::getJointPlanePoseInWorldFrame() const {
    // 計算從 WORLD_FRAME 到 BED_JOINT_PLANE_FRAME 的總變換矩陣
    Eigen::Affine3d total_transform = getTransform("WORLD_FRAME", "NOZZLE_FRAME") *
                                      getTransform("PART_FRAME", "NOZZLE_FRAME").inverse() *
                                      getTransform("WORKPIECE_ORIGIN_FRAME", "PART_FRAME").inverse() *
                                      getTransform("BED_SURFACE_FRAME", "WORKPIECE_ORIGIN_FRAME").inverse() *
                                      getTransform("BED_JOINT_PLANE_FRAME", "BED_SURFACE_FRAME").inverse();

    // 提取平移分量
    Eigen::Vector3d translation = total_transform.translation();

    // 提取旋轉分量並轉換為歐拉角 (ZYX 順序)
//    Eigen::Vector3d euler_angles = total_transform.rotation().eulerAngles(2, 1, 0); // ZYX 順序

    Eigen::Matrix3d R = total_transform.rotation();  // 獲取旋轉矩陣
    double roll = atan2(R(2, 1), R(2, 2));  // 繞 X 軸的旋轉角
    double pitch = asin(-R(2, 0));          // 繞 Y 軸的旋轉角
    double yaw = atan2(R(1, 0), R(0, 0));   // 繞 Z 軸的旋轉角

    // 將位移和旋轉角度存入 SPPose 結構
    SPPose pose;
    pose.x = translation.x();
    pose.y = translation.y();
    pose.z = translation.z();
//    pose.phi = euler_angles[2];   // Roll (phi)
//    pose.theta = euler_angles[1]; // Pitch (theta)
//    pose.psi = euler_angles[0];   // Yaw (psi)
    pose.phi = roll;
    pose.theta = pitch;
    pose.psi = yaw;

    return pose;
}

// 用於設置 part2nozzleTransform 的新函數
void TFTransformer::setPartToNozzleTransform(double tx, double ty, double tz, double rx_degree, double ry_degree, double rz_degree) {
    Eigen::Quaterniond rotation = Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(rx_degree), Eigen::Vector3d::UnitX())) *
                                  Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(ry_degree), Eigen::Vector3d::UnitY())) *
                                  Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(rz_degree), Eigen::Vector3d::UnitZ()));
    part2nozzleTransform = Eigen::Translation3d(tx, ty, tz) * rotation;
}

// 用於設置 workpiece_origin2partTransform 的新函數
void TFTransformer::setWorkpieceOriginToPartTransform(double tx, double ty, double tz, double rx_degree, double ry_degree, double rz_degree) {
    Eigen::Quaterniond rotation = Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(rx_degree), Eigen::Vector3d::UnitX())) *
                                  Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(ry_degree), Eigen::Vector3d::UnitY())) *
                                  Eigen::Quaterniond(Eigen::AngleAxisd(degreeToRad(rz_degree), Eigen::Vector3d::UnitZ()));
    workpiece_origin2partTransform = Eigen::Translation3d(tx, ty, tz) * rotation;
}

