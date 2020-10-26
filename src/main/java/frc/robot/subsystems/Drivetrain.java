/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.DriveTrainVelocityControlConstantsLeft;
import frc.robot.Constants.DriveTrainVelocityControlConstantsRight;
import frc.robot.Constants.DriverConstants;

public class Drivetrain extends SubsystemBase {
  private static int[] DTL_IDs = { //Drive Train Left
    DriveTrainConstants.DTL_FRONT, DriveTrainConstants.DTL_BACK
  };

  private static int[] DTR_IDs = { //Drive Train Left
    DriveTrainConstants.DTR_FRONT, DriveTrainConstants.DTR_BACK
  };

  private CANSparkMax FrontLeft, BackLeft, FrontRight, BackRight;

  private CANEncoder FrontRightEnc, FrontLeftEnc;

  private CANEncoder BackRightEnc, BackLeftEnc;

  private double x, y;
  private double leftVelocity, rightVelocity;

  private AHRS navx;

  private DifferentialDrive robot;

  private DifferentialDriveKinematics kinematics;
  private ChassisSpeeds chassisSpeeds;
  private DifferentialDriveWheelSpeeds wheelSpeeds;

  private double startTime;

  private RamseteController autoController;
  private Trajectory.State goal;
  private Trajectory trajectory;
  private Trajectories trajectories;
  private Pose2d currentPose;
  private DifferentialDriveOdometry odometer;
  private Rotation2d yaw;

  private PowerDistributionPanel pdp;

  private CANPIDController velocityControllerLeft;
  private CANPIDController velocityControllerRight;

  public Drivetrain() {
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

    velocityControllerLeft = FrontLeft.getPIDController();
    
    velocityControllerLeft.setP(DriveTrainVelocityControlConstantsLeft.kP);
    velocityControllerLeft.setI(DriveTrainVelocityControlConstantsLeft.kI);
    velocityControllerLeft.setD(DriveTrainVelocityControlConstantsLeft.kD);
    velocityControllerLeft.setIZone(DriveTrainVelocityControlConstantsLeft.kIZone);
    velocityControllerLeft.setFF(DriveTrainVelocityControlConstantsLeft.kFF);

    velocityControllerRight = FrontRight.getPIDController();
    
    velocityControllerRight.setP(DriveTrainVelocityControlConstantsRight.kP);
    velocityControllerRight.setI(DriveTrainVelocityControlConstantsRight.kI);
    velocityControllerRight.setD(DriveTrainVelocityControlConstantsRight.kD);
    velocityControllerRight.setIZone(DriveTrainVelocityControlConstantsRight.kIZone);
    velocityControllerRight.setFF(DriveTrainVelocityControlConstantsRight.kFF);

    FrontLeft.burnFlash();
    BackLeft.burnFlash();
    FrontRight.burnFlash();
    BackRight.burnFlash();

    //END SPARK MAX FLASH SETTINGS --------------------------

    trajectories = new Trajectories();

    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(30.0));
    chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    trajectory = trajectories.getStraightTrajectory();

    autoController = new RamseteController(2.0, 0.7);
    
    //robot = new DifferentialDrive(FrontLeft, FrontRight);

    navx = new AHRS(Constants.NAVX_PORT);
    navx.reset();

    pdp = new PowerDistributionPanel();

    yaw = new Rotation2d(Units.degreesToRadians(navx.getYaw()));
    odometer = new DifferentialDriveOdometry(yaw, new Pose2d(0, 0, new Rotation2d()));
  }

  public void ControllerSplitArcade(XboxController joy) {
    y = joy.getY(Hand.kLeft);
    x = joy.getX(Hand.kRight);

    robot.arcadeDrive(-y * 1, x * 1);
  }

  public void ClosedLoopControllerSplitArcade(XboxController joy) { //Side-side
    y = joy.getY(Hand.kLeft);
    x = joy.getX(Hand.kRight);

    if(!joy.getBumper(Hand.kLeft)) {
      x *= -y;
    }
    
    //3.5 m/s at maximum speed, 2 rad/s maximum rotation rate (~114 deg/s)
    chassisSpeeds = new ChassisSpeeds(-y * DriverConstants.maxSpeed, 0.0, -x * DriverConstants.maxRotation);

    wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    leftVelocity = wheelSpeeds.leftMetersPerSecond;
    rightVelocity = -wheelSpeeds.rightMetersPerSecond;

    //Convert m/s to rpm:
    leftVelocity = (leftVelocity * 60 * DriveTrainConstants.reduction) / DriveTrainConstants.wheelCircumference;
    rightVelocity = (rightVelocity * 60 * DriveTrainConstants.reduction) / DriveTrainConstants.wheelCircumference;

    velocityControllerLeft.setReference(leftVelocity, ControlType.kVelocity);
    velocityControllerRight.setReference(rightVelocity, ControlType.kVelocity);

    SmartDashboard.putNumber("Left Error", getFrontLeftRPM() - leftVelocity);
    SmartDashboard.putNumber("Right Error", getFrontRightRPM() - rightVelocity);
  }

  public void ClosedLoopControllerSplitArcadeYawDrive(XboxController joy) { //Yaw-Drive
    y = joy.getY(Hand.kLeft);
    x = joy.getX(Hand.kRight);

    if(!joy.getBumper(Hand.kLeft)) {
      x *= -y;
    }
    
    //3.5 m/s at maximum speed, 2 rad/s maximum rotation rate (~114 deg/s)
    chassisSpeeds = new ChassisSpeeds(-y * DriverConstants.maxSpeed, 0.0, -x * DriverConstants.maxRotation);

    wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    leftVelocity = wheelSpeeds.leftMetersPerSecond;
    rightVelocity = -wheelSpeeds.rightMetersPerSecond;

    //Convert m/s to rpm:
    leftVelocity = (leftVelocity * 60 * DriveTrainConstants.reduction) / DriveTrainConstants.wheelCircumference;
    rightVelocity = (rightVelocity * 60 * DriveTrainConstants.reduction) / DriveTrainConstants.wheelCircumference;

    FrontRight.set(rightVelocity * DriveTrainVelocityControlConstantsRight.kFF);
    BackRight.set(rightVelocity * DriveTrainVelocityControlConstantsRight.kFF);
    FrontLeft.set(leftVelocity * DriveTrainVelocityControlConstantsLeft.kFF);
    BackLeft.set(leftVelocity * DriveTrainVelocityControlConstantsLeft.kFF);
  }

  public void stopClosedLoop() {
    velocityControllerLeft.setReference(0, ControlType.kVelocity);
    velocityControllerRight.setReference(0, ControlType.kVelocity);
  }

  public void stopMotors() {
    robot.stopMotor();
  }

  public double getFrontRightRPM(){
    return FrontRightEnc.getVelocity();
  }

  public double getFrontLeftRPM(){
    return FrontLeftEnc.getVelocity();
  }

  public double getBackRightRPM(){
    return BackRightEnc.getVelocity();
  }

  public double getBackLeftRPM(){
    return BackLeftEnc.getVelocity();
  }

  public void pathFollow() {
    currentPose = odometer.getPoseMeters();
    
    SmartDashboard.putNumber("AutoTime", Timer.getFPGATimestamp() - startTime);
    goal = trajectory.sample(Timer.getFPGATimestamp() - startTime); // sample the trajectory at time seconds from the beginning

    chassisSpeeds = autoController.calculate(currentPose, goal);

    wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    leftVelocity = wheelSpeeds.leftMetersPerSecond;
    rightVelocity = -wheelSpeeds.rightMetersPerSecond;

    //Convert m/s to rpm:
    leftVelocity = (leftVelocity * 60 * DriveTrainConstants.reduction) / DriveTrainConstants.wheelDiameter;
    rightVelocity = (rightVelocity * 60 * DriveTrainConstants.reduction) / DriveTrainConstants.wheelDiameter;

    velocityControllerLeft.setReference(leftVelocity, ControlType.kVelocity);
    velocityControllerRight.setReference(rightVelocity, ControlType.kVelocity);
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

  public void resetEncoders() {
    FrontLeftEnc.setPosition(0);
    FrontRightEnc.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Total Current", pdp.getTotalCurrent());
    SmartDashboard.putNumber("FrontRightRPM", getFrontRightRPM());
    SmartDashboard.putNumber("FrontLeftRPM", getFrontLeftRPM());

    SmartDashboard.putNumber("FrontRightPos", -FrontRightEnc.getPosition());
    SmartDashboard.putNumber("FrontLeftPos", FrontLeftEnc.getPosition());

    updateOdometry();
  }
}