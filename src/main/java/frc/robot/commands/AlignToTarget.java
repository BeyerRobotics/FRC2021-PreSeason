/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Odometry;

public class AlignToTarget extends CommandBase {
  /**
   * Creates a new AlignToTarget.
   */

  private Drivetrain drivetrain;
  private Odometry odometry;
  private Limelight limelight;

  private double angle = 0;

  public AlignToTarget(Drivetrain drivetrainSubsystem, Odometry odometrySubsystem, Limelight limelightSubsystem) {
    drivetrain = drivetrainSubsystem;
    odometry = odometrySubsystem;
    limelight = limelightSubsystem;

    addRequirements(drivetrain);
    addRequirements(odometry);
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.validTarget()){
      drivetrain.TurnToAnglePID(angle, 1.0, "limelight");
    }else{
      angle = odometry.getAngleToTarget();
      drivetrain.TurnToAnglePID(angle, 1.0, "odometry");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
