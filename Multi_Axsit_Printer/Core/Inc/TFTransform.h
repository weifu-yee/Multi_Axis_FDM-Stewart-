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


class TFTransformer {
public:
    TFTransformer(); // 構造函數用於初始化變換矩陣
    Eigen::Affine3d getTransform(const std::string& from, const std::string& to) const;
    Eigen::Affine3d getJointPlanePoseInWorldFrame() const;

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
