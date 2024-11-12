/*
 * TFTransform.cpp
 *
 *  Created on: Nov 13, 2024
 *      Author: user
 */

#include "TFTransform.h"
#include <Eigen/Geometry>
#include "constants.h"

// 將角度轉換為弧度的函數
double degreeToRad(double angle_in_degrees) {
    return angle_in_degrees * M_PI / 180.0;
}

TFTransformer::TFTransformer()
    : word2nozzleTransform((Eigen::Translation3d(WORD2NOZZLE_TRANSLATION_X, WORD2NOZZLE_TRANSLATION_Y, WORD2NOZZLE_TRANSLATION_Z) *
                            Eigen::AngleAxisd(degreeToRad(WORD2NOZZLE_ROTATION_X_DEGREE), Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(degreeToRad(WORD2NOZZLE_ROTATION_Y_DEGREE), Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(degreeToRad(WORD2NOZZLE_ROTATION_Z_DEGREE), Eigen::Vector3d::UnitZ())).matrix()),

      bed_surface2workpiece_originTransform((Eigen::Translation3d(BED_SURFACE2WORKPIECE_ORIGIN_TRANSLATION_X, BED_SURFACE2WORKPIECE_ORIGIN_TRANSLATION_Y, BED_SURFACE2WORKPIECE_ORIGIN_TRANSLATION_Z) *
                                            Eigen::AngleAxisd(degreeToRad(BED_SURFACE2WORKPIECE_ORIGIN_ROTATION_X_DEGREE), Eigen::Vector3d::UnitX()) *
                                            Eigen::AngleAxisd(degreeToRad(BED_SURFACE2WORKPIECE_ORIGIN_ROTATION_Y_DEGREE), Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(degreeToRad(BED_SURFACE2WORKPIECE_ORIGIN_ROTATION_Z_DEGREE), Eigen::Vector3d::UnitZ())).matrix()),

      bed_joint_plane2bed_surfaceTransform((Eigen::Translation3d(BED_JOINT_PLANE2BED_SURFACE_TRANSLATION_X, BED_JOINT_PLANE2BED_SURFACE_TRANSLATION_Y, BED_JOINT_PLANE2BED_SURFACE_TRANSLATION_Z) *
                                           Eigen::AngleAxisd(degreeToRad(BED_JOINT_PLANE2BED_SURFACE_ROTATION_X_DEGREE), Eigen::Vector3d::UnitX()) *
                                           Eigen::AngleAxisd(degreeToRad(BED_JOINT_PLANE2BED_SURFACE_ROTATION_Y_DEGREE), Eigen::Vector3d::UnitY()) *
                                           Eigen::AngleAxisd(degreeToRad(BED_JOINT_PLANE2BED_SURFACE_ROTATION_Z_DEGREE), Eigen::Vector3d::UnitZ())).matrix()) {
    // 初始化非 const 變換矩陣
	part2nozzleTransform = Eigen::Translation3d(1.0, 1.0, 0.0) *
	                       Eigen::AngleAxisd(degreeToRad(0.0), Eigen::Vector3d::UnitX()) *
	                       Eigen::AngleAxisd(degreeToRad(0.0), Eigen::Vector3d::UnitY()) *
	                       Eigen::AngleAxisd(degreeToRad(90.0), Eigen::Vector3d::UnitZ());
	workpiece_origin2partTransform = Eigen::Translation3d(0.5, 0.5, 0.5) *
	                                 Eigen::AngleAxisd(degreeToRad(0.0), Eigen::Vector3d::UnitX()) *
	                                 Eigen::AngleAxisd(degreeToRad(45.0), Eigen::Vector3d::UnitY()) *
	                                 Eigen::AngleAxisd(degreeToRad(0.0), Eigen::Vector3d::UnitZ());
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

// 用於設置 part2nozzleTransform 的新函數
void TFTransformer::setPartToNozzleTransform(double tx, double ty, double tz, double rx_degree, double ry_degree, double rz_degree) {
    part2nozzleTransform = Eigen::Translation3d(tx, ty, tz) *
                           Eigen::AngleAxisd(degreeToRad(rx_degree), Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(degreeToRad(ry_degree), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(degreeToRad(rz_degree), Eigen::Vector3d::UnitZ());
}

// 用於設置 workpiece_origin2partTransform 的新函數
void TFTransformer::setWorkpieceOriginToPartTransform(double tx, double ty, double tz, double rx_degree, double ry_degree, double rz_degree) {
    workpiece_origin2partTransform = Eigen::Translation3d(tx, ty, tz) *
                                     Eigen::AngleAxisd(degreeToRad(rx_degree), Eigen::Vector3d::UnitX()) *
                                     Eigen::AngleAxisd(degreeToRad(ry_degree), Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(degreeToRad(rz_degree), Eigen::Vector3d::UnitZ());
}

