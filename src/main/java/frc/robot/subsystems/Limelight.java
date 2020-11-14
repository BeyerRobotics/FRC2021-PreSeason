/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Limelight extends SubsystemBase {

  private Odometry odometry;

  private double[] LookUpTableInches = Constants.RPMConstants.inches;
  private double[] LookUpTableRPM = Constants.RPMConstants.RPM;

  private double moduloResult;
  private double slope;

  public NetworkTable table;
  public NetworkTableEntry pipeline, camMode, ledMode, tx, ty, ta, tv;

  public double distance, cameraAngle = 17.9, cameraHeight = 23.5, targetHeight = 90;

  public double horizontalSetPoint = 0, verticalSetPoint;

  private double Kp = 0.00490, Ki = 0.00027, Kd, integral, derivative;

  private double Kpy = 0.0065, Kiy = 0.000, Kdy = 0, integralY, derivativeY;

  private double error, prevError, prevErrory, errory, steeringAdjust, steeringAdjustV;

  private double min,minY, max, leftDriveCalc, rightDriveCalc, kMin = 0.018, kMinY = 0.015;

  public double leftPower, rightPower;

  public double a = 0.000013057723362082, b = -.013184895991886, c = 4.9843672185983, d = -832.91901660932, e = 58836.549704051;

  public double hRes;

  public boolean preValid = false;

  public Limelight(Odometry subsystem) {
    odometry = subsystem;
    horizontalSetPoint = 0;
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public double getRPM(double distance) {
    distance -= 96; //Normalize such that at 8 feet distance is 0, since that's the first point in our data
    moduloResult = distance % 8; //Find how far between 8 inch intervals we are
    int floorPoint = (int)Math.floor((distance - moduloResult) / 8); //Find point in table just below the inputted distance
    int roofPoint = (int)Math.floor((distance + (8 - moduloResult)) / 8); //Find point in table above the inputted distance

    slope = (LookUpTableRPM[roofPoint] - LookUpTableRPM[floorPoint]) / (LookUpTableInches[roofPoint] - LookUpTableInches[floorPoint]); //Calculate slope between points

    return (LookUpTableRPM[floorPoint] + slope * moduloResult); //Calculate output based on point just below inputted distance + slope times how far between the points you are
  }

  public double getHorizontalOffset() {
    return tx.getDouble(0.0);
  }

  public double getHorizontalOffsetInDegrees(){
    double vpw = 2.0*(Math.tan(Constants.limelightConstants.hFOV/2));

    double nx = getHorizontalOffset() * (1/(hRes/2));

    double x = (vpw/2)*nx;

    return Math.atan(x);
  }

  public double getVerticalOffset() {
    return ty.getDouble(0.0);
  }

  public double getTargetArea() {
    return ta.getDouble(0.0);
  }

  public boolean validTarget() {
    if (tv.getDouble(0.0) == 1.0) {
      return true;
    } else {
      return false;
    }
  }

  
  public double getDistance() {
    final double angle = cameraAngle + getVerticalOffset();
    distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));
    return distance;
  }

  public void validOdometryReset(){

    if(validTarget() && !preValid && Math.abs(getHorizontalOffset()) < 5){
      odometry.offsetOdometryLimeLight(getDistance());
      preValid = true;
    }

    if(!validTarget()){
      preValid = false;
    }

  }

  public void seekPipeline(){
    if(validTarget() && Math.abs(getHorizontalOffset()) < 100){
      setPipeline(1);
      hRes = 320;
    }else{
      setPipeline(2);
      hRes = 720;
    }
  }

  public void getLimeLightSmartDashboard(){
    SmartDashboard.putNumber("HorizontalOffset", getHorizontalOffset());
    SmartDashboard.putNumber("VerticalOffset", getVerticalOffset());
    SmartDashboard.putNumber("TargetArea", getTargetArea());
    SmartDashboard.putBoolean("ValidTarget", validTarget());
    SmartDashboard.putNumber("IntegralX", integral);
    SmartDashboard.putNumber("IntegralY", integralY);
    SmartDashboard.putNumber("distance", getDistance());
  }

  public void setPipeline(int pipeline){
    NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    pipelineEntry.setNumber(pipeline);
  }

  @Override
  public void periodic() {
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");
  
    validOdometryReset();
    seekPipeline();
    getLimeLightSmartDashboard();
  }
}