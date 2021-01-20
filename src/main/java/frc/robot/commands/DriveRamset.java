// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveRamset extends RamseteCommand {

  public DriveRamset(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController follower,
      DifferentialDriveKinematics kinematics, BiConsumer<Double, Double> outputMetersPerSecond,
      Subsystem[] requirements) {
    super(trajectory, pose, follower, kinematics, outputMetersPerSecond, requirements);
    // TODO Auto-generated constructor stub
  }
  
}
