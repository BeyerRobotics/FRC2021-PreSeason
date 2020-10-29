/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DTVelocityControllersPID extends SubsystemBase {
  private double state = 0.0;
  private double prevState = 0;
  private double setpoint = 0.0;
  private double kP = 0, kI = 0, kD = 0, kF = 0, kIZone = 0;
  private double accumulator = 0;

  public DTVelocityControllersPID() {

  }

  public void setParams(double ckP, double ckI, double ckD, double ckF, double ckIZone) {
    kP = ckP;
    kI = ckI;
    kD = ckD;
    kF = ckF;
    kIZone = ckIZone;
  }

  public void set(double setting) {
    setpoint = setting;
  }

  public void update(double input) {
    state = input;
  }

  public double calculate() { //Where the magic happens
    double output = 0.0;
    double error = state - setpoint; //Error is negative when state is below setpoint
    double oP = 0, oI = 0, oD = 0, oF = 0;
    double deltaState = state - prevState; //DeltaState is negative when state is lower than previous state

    //Calculate the Proportional Term
    oP = error * kP;
    
    //Calculate the Integral Term
    if(Math.abs(error) > kIZone && kIZone != 0) {
      accumulator = 0; //Reset accumulator when outside IZone if IZone isn't 0
    } else {
      accumulator -= error; //Accumulator should grow large and positive when error is negative (ie below setpoint)
    }
    oI = accumulator * kI;

    //Calculate the Derivative Term
    oD = -deltaState * kD; //When the state is changing, D output should fight it (ie state decreasing, deltaState negative, oD positive)
    
    //Calculate the FeedForward Term
    oF = setpoint * kF;

    output = oP + oI + oD + oF;
    return output;
  }

  @Override
  public void periodic() {

  }
}