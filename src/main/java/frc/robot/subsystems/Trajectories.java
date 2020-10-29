/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;

public class Trajectories extends SubsystemBase {

  private Trajectory testTrajectory;
  private Trajectory straightTrajectory;
  private Trajectory figureEightTrajectory;

  public Trajectories() {
    // Initiate Trajectories
    initTestTrajectory();
    initStraightTrajectory();
    initFigureEightTrajectory();
  }

  public void initTestTrajectory() {

    // 2018 cross scale auto waypoints.
    var sideStart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23), Rotation2d.fromDegrees(-180));
    var crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8), Rotation2d.fromDegrees(-160));

    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
    config.setReversed(false);

    testTrajectory = TrajectoryGenerator.generateTrajectory(sideStart, interiorWaypoints, crossScale, config);
  }

  public void initStraightTrajectory() {
    final String trajectoryJSON = "paths/output/Straight.wpilib.json";
    final Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    try {
      straightTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void initFigureEightTrajectory() {
    final String trajectoryJSON = "paths/output/Figure Eight.wpilib.json";
    final Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    try {
      figureEightTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public Trajectory getStraightTrajectory() {
    return straightTrajectory;
  }

  public Trajectory getFigureEightTrajectory() {
    return figureEightTrajectory;
  }

  public Trajectory getTestTrajectory() {
    return testTrajectory;
  }

  @Override
  public void periodic() {

  }
}