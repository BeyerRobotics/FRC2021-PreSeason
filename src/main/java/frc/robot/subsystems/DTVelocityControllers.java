/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DTVelocityControllers extends SubsystemBase {
  DTVelocityControllersLoop DTControllers = new DTVelocityControllersLoop();
  double leftVelocity = 0;
  double rightVelocity = 0;

  public DTVelocityControllers() {
    DTControllers.initPIDs();
    DTControllers.start();
  }

  public void setLeftVelocity(double velocity) { //Ask josh if these are necessary
    DTControllers.setLeftVelocity(velocity);
  }

  public void setRightVelocity(double velocity) {
    DTControllers.setRightVelocity(velocity);
  }

  @Override
  public void periodic() {

  }
}