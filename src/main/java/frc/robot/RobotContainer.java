/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ClosedLoopXboxControllerSplitArcade;
//import frc.robot.commands.XboxControllerSplitArcade;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Trajectories;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  XboxController Driver = new XboxController(ControllerConstants.DRIVER);

  private final Drivetrain drivetrain = new Drivetrain();

  //private final XboxControllerSplitArcade xboxControllerSplitArcade = new XboxControllerSplitArcade(drivetrain, Driver);

  private final ClosedLoopXboxControllerSplitArcade closedLoopXboxControllerSplitArcade = new ClosedLoopXboxControllerSplitArcade(drivetrain, Driver);

  private final Trajectories trajectories = new Trajectories();
  private final AutonomousCommand autoCommand = new AutonomousCommand(drivetrain, trajectories);


  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(Driver, 6).whenHeld(closedLoopXboxControllerSplitArcade);
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
