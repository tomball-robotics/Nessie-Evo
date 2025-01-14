// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class ASUtil {
    public static double[] pose3dToDoubleArray(Pose3d pose){
        return new double[] {
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        pose.getRotation().getQuaternion().getW(),
        pose.getRotation().getQuaternion().getX(),
        pose.getRotation().getQuaternion().getY(),
        pose.getRotation().getQuaternion().getZ()};
    }
}
