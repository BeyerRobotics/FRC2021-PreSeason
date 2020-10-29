/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;

public class DTVelocityControllersLoop extends Thread {
  DTVelocityControllersPID drivetrainLeft = new DTVelocityControllersPID();
  DTVelocityControllersPID drivetrainRight = new DTVelocityControllersPID();
  long sleepTime = 5; //Milliseconds, 200hz
  Drivetrain dt =  new Drivetrain(); //ASK JOSH IF THIS IS ILLEGAL
  private double leftVelocitySetpoint = 0;
  private double rightVelocitySetpoint = 0;
  private double leftOutput = 0;
  private double rightOutput = 0;

  public void initPIDs() {
    drivetrainLeft.setParams(Constants.DriveTrainVelocityControlConstants.kP, Constants.DriveTrainVelocityControlConstants.kI, Constants.DriveTrainVelocityControlConstants.kD, Constants.DriveTrainVelocityControlConstants.kFF, Constants.DriveTrainVelocityControlConstants.kIZone);
    drivetrainRight.setParams(Constants.DriveTrainVelocityControlConstants.kP, Constants.DriveTrainVelocityControlConstants.kI, Constants.DriveTrainVelocityControlConstants.kD, Constants.DriveTrainVelocityControlConstants.kFF, Constants.DriveTrainVelocityControlConstants.kIZone);
  }

  public void setLeftVelocity(double leftVelocity) {
    leftVelocitySetpoint = leftVelocity;
  }

  public void setRightVelocity(double rightVelocity) {
    rightVelocitySetpoint = rightVelocity;
  }

  public void run() {
    drivetrainLeft.update(dt.getLeftVelocity());
    drivetrainRight.update(dt.getRightVelocity());

    drivetrainLeft.set(leftVelocitySetpoint);
    drivetrainRight.set(rightVelocitySetpoint);

    leftOutput = drivetrainLeft.calculate();
    rightOutput = drivetrainRight.calculate();

    dt.setLeftMotors(leftOutput);
    dt.setRightMotors(rightOutput);

    try {
      Thread.sleep(sleepTime);
    } catch(InterruptedException e) {
      System.out.println("PID Thread Interrupted");
    }
  }
}