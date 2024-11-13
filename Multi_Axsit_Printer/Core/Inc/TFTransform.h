/*
 * TFTransform.h
 *
 *  Created on: Nov 13, 2024
 *      Author: user
 */

#ifndef INC_TFTRANSFORM_H_
#define INC_TFTRANSFORM_H_

#include <Eigen/Dense>
#include <string>
#include "stewart_platform.h"

class TFTransformer {
public:
    TFTransformer(); // 構造函數用於初始化變換矩陣
    Eigen::Affine3d getTransform(const std::string& from, const std::string& to) const;
    SPPose getJointPlanePoseInWorldFrame() const; // 更新返回類型為 SPPose

    // 新增公共函數來更新非 const 變換矩陣
    void setPartToNozzleTransform(double tx, double ty, double tz, double rx_degree, double ry_degree, double rz_degree);
    void setWorkpieceOriginToPartTransform(double tx, double ty, double tz, double rx_degree, double ry_degree, double rz_degree);

private:
    // 變換矩陣的成員變數
    const Eigen::Affine3d word2nozzleTransform;
    const Eigen::Affine3d bed_surface2workpiece_originTransform;
    const Eigen::Affine3d bed_joint_plane2bed_surfaceTransform;

    // 非 const 的變換矩陣
    Eigen::Affine3d part2nozzleTransform;
    Eigen::Affine3d workpiece_origin2partTransform;
};

#endif /* INC_TFTRANSFORM_H_ */
