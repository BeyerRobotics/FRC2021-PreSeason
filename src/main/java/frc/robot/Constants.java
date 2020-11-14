/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public class DriveTrainConstants {
    //Left Motor Ports
    public static final int DTL_FRONT = 1;
    public static final int DTL_BACK = 2;

    //Right Motor Ports
    public static final int DTR_FRONT = 3;
    public static final int DTR_BACK = 4;

    //Physical Characteristics (in meters)
    public static final double wheelDiameter = 0.1524;
    public static final double wheelCircumference = 0.47878;
    public static final double reduction = 6.67;
  }

  public static class RPMConstants {
    public static final double[] inches = {0.0, 1.0, 2.0};
    public static final double[] RPM = {100.0, 200.0, 300.0};
  }

  public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;

  public class ControllerConstants {
    //Joysticks
    public static final int DRIVER = 0;
    public static final int GUNNER = 1;
  }

  public class DriverConstants {
    public static final double maxSpeed = 3.5;  //  m/s
    public static final double maxRotation = 4; //  rad/s

     /*Navx PID*/
     public static final double Kp = 0.005;//0.00025
     public static final double Ki = 0.00000;//0.00000165
     public static final double Kd = 0.0006;//0.13
     public static final double FF = 0.0;//.014
     public static final double IZone = 100;//100
     public static final double Min = 0;
     public static final double Max = 1;
  }

  public class DriveTrainVelocityControlConstantsLeft {
    public static final double kP = 0.0008;
    public static final double kI = 0.000001;
    public static final double kD = 0.001; //0.001 works well
    public static final double kIZone = 100;
    public static final double kFF = 0.00019;
  }

  public class DriveTrainVelocityControlConstantsRight {
    public static final double kP = 0.0008; //0.0008
    public static final double kI = 0.000001; //0.000001
    public static final double kD = 0.001;
    public static final double kIZone = 100;
    public static final double kFF = 0.00019;
  }

  public class fieldParameters{
    public static final double targetY  = 1;
    public static final double targetX  = 1;
  }

  public class limelightConstants{
    public static final double vFOV = 41;
    public static final double hFOV = 54;
  }
}
