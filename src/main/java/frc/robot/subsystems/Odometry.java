/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.fieldParameters;

public class Odometry extends SubsystemBase {
  /**
   * Creates a new Odometry.
   */

  private static int[] DTL_IDs = { //Drive Train Left
    DriveTrainConstants.DTL_FRONT, DriveTrainConstants.DTL_BACK
  };

  private static int[] DTR_IDs = { //Drive Train Left
    DriveTrainConstants.DTR_FRONT, DriveTrainConstants.DTR_BACK
  };

  private AHRS navx;
  
  private Pose2d currentPose;
  private DifferentialDriveOdometry odometer;
  private Rotation2d yaw;

  public CANSparkMax FrontLeft, BackLeft, FrontRight, BackRight;

  public CANEncoder FrontRightEnc, FrontLeftEnc;

  public CANEncoder BackRightEnc, BackLeftEnc;

  private double angle;

  public double startTime;

  
  public Odometry() {
    navx = new AHRS(Constants.NAVX_PORT); 
    navx.reset();

    yaw = new Rotation2d(Units.degreesToRadians(navx.getYaw())); //Remember its standard convention for left to be positive 
    odometer = new DifferentialDriveOdometry(yaw, new Pose2d(0, 0, new Rotation2d()));

    FrontLeft = new CANSparkMax(DTL_IDs[0], MotorType.kBrushless);
    BackLeft = new CANSparkMax(DTL_IDs[1], MotorType.kBrushless);
    FrontRight = new CANSparkMax(DTR_IDs[0], MotorType.kBrushless);
    BackRight = new CANSparkMax(DTR_IDs[1], MotorType.kBrushless);


    FrontLeft.restoreFactoryDefaults();
    BackLeft.restoreFactoryDefaults();
    FrontRight.restoreFactoryDefaults();
    BackRight.restoreFactoryDefaults();

    FrontLeft.setIdleMode(IdleMode.kBrake);
    BackLeft.setIdleMode(IdleMode.kBrake);
    FrontRight.setIdleMode(IdleMode.kBrake);
    BackRight.setIdleMode(IdleMode.kBrake);

    BackLeft.follow(FrontLeft);
    BackRight.follow(FrontRight);

    FrontRightEnc = FrontRight.getEncoder();
    FrontLeftEnc = FrontLeft.getEncoder();
    BackRightEnc = BackRight.getEncoder();
    BackLeftEnc = BackLeft.getEncoder();

    FrontRightEnc.setPositionConversionFactor(DriveTrainConstants.wheelCircumference/DriveTrainConstants.reduction);
    FrontLeftEnc.setPositionConversionFactor(DriveTrainConstants.wheelCircumference/DriveTrainConstants.reduction);
    BackRightEnc.setPositionConversionFactor(DriveTrainConstants.wheelCircumference/DriveTrainConstants.reduction);
    BackLeftEnc.setPositionConversionFactor(DriveTrainConstants.wheelCircumference/DriveTrainConstants.reduction);

    BackLeft.setSmartCurrentLimit(25);
    BackRight.setSmartCurrentLimit(25);
    FrontLeft.setSmartCurrentLimit(25);
    FrontRight.setSmartCurrentLimit(25);

    BackLeft.enableVoltageCompensation(12);
    BackRight.enableVoltageCompensation(12);
    FrontLeft.enableVoltageCompensation(12);
    FrontRight.enableVoltageCompensation(12);

    BackLeft.setClosedLoopRampRate(0.05);
    BackRight.setClosedLoopRampRate(0.05);
    FrontLeft.setClosedLoopRampRate(0.05);
    FrontRight.setClosedLoopRampRate(0.05);
  }

  public void resetOdometry() {
    startTime = Timer.getFPGATimestamp();

    navx.reset();

    resetEncoders();

    Pose2d initPose = new Pose2d(0.0, 0.0, new Rotation2d());

    odometer.resetPosition(initPose, new Rotation2d());
  }

  public void updateOdometry() {
    var gyroAngle = Rotation2d.fromDegrees(-navx.getAngle());

    // Update the pose
    currentPose = odometer.update(gyroAngle, FrontLeftEnc.getPosition(), -FrontRightEnc.getPosition());
    SmartDashboard.putNumber("Yaw", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("X", currentPose.getTranslation().getX());
    SmartDashboard.putNumber("Y", currentPose.getTranslation().getY());
  }

  public double getAngleToTarget(){
    double dty, dtx;
    double theta;

    dty = currentPose.getTranslation().getY() - Constants.fieldParameters.targetY;
    dtx = currentPose.getTranslation().getX() - Constants.fieldParameters.targetX;

    theta = -Math.atan(dty/dtx);

    return theta;
  }

   /**
   * Resets odometry based on limelight data.
   * @param distance is in meters
   */

  public void offsetOdometryLimeLight(double distance){
    double dty, dtx, dpy, dpx, theta, xp, yp;

    dty = currentPose.getTranslation().getY() - Constants.fieldParameters.targetY;
    dtx = currentPose.getTranslation().getX() - Constants.fieldParameters.targetX;

    theta = -Math.atan(dty/dtx);

    dpy = Math.sin(theta)*distance;
    dpx = Math.cos(theta)*distance; 

    yp = dpy + fieldParameters.targetY;
    xp = dpx + fieldParameters.targetX;

    odometer.resetPosition(new Pose2d(yp,xp, new Rotation2d(-theta)), new Rotation2d(-navx.getYaw()));
    resetEncoders();
  }

  public void resetEncoders() {
    FrontLeftEnc.setPosition(0);
    FrontRightEnc.setPosition(0);
  }

  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  public double getYaw(){
    return odometer.getPoseMeters().getRotation().getDegrees();
  }

  public double getFrontRightRPM(){
    return FrontRightEnc.getVelocity();
  }

  public double getFrontLeftRPM(){
    return FrontLeftEnc.getVelocity();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
