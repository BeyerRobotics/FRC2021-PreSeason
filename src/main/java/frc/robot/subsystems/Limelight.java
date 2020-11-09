/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Limelight extends SubsystemBase {

  private double[] LookUpTableInches = Constants.RPMConstants.inches;
  private double[] LookUpTableRPM = Constants.RPMConstants.RPM;

  private double moduloResult;
  private double slope;


  public Limelight() {
    //hehe
  }

  public double getRPM(double distance) {
    distance -= 96; //Normalize such that at 8 feet distance is 0, since that's the first point in our data
    moduloResult = distance % 8; //Find how far between 8 inch intervals we are
    int floorPoint = (int)Math.floor((distance - moduloResult) / 8); //Find point in table just below the inputted distance
    int roofPoint = (int)Math.floor((distance + (8 - moduloResult)) / 8); //Find point in table above the inputted distance

    slope = (LookUpTableRPM[roofPoint] - LookUpTableRPM[floorPoint]) / (LookUpTableInches[roofPoint] - LookUpTableInches[floorPoint]); //Calculate slope between points

    return (LookUpTableRPM[floorPoint] + slope * moduloResult); //Calculate output based on point just below inputted distance + slope times how far between the points you are
  }

  @Override
  public void periodic() {

  }
}