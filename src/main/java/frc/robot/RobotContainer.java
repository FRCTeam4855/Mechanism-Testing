// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;

	import static edu.wpi.first.units.Units.Degrees;

public class RobotContainer {
    private final ArmSubsystem m_armSubsystem;

    private final GenericHID m_controller =
      new GenericHID(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {
    m_armSubsystem = new ArmSubsystem();
    configureBindings();
    // Set the default command to force the arm to go to 0.
    m_armSubsystem.setDefaultCommand(m_armSubsystem.setAngle(Degrees.of(0)));    
  }

  private void configureBindings() {

    //Pre-Configured Setpoints
    new JoystickButton(m_controller, 1).onTrue(m_armSubsystem.setAngle(Degrees.of(90)));
    new JoystickButton(m_controller, 2).onTrue(m_armSubsystem.setAngle(Degrees.of(0)));

    //Manual Setpoint from Smartdashboard
    new JoystickButton(m_controller, 3).onTrue(m_armSubsystem.setAngle(Degrees.of(SmartDashboard.getNumber("Arm Setpoint", 0))));

    //Manual Percent Output
    new JoystickButton(m_controller, 4).whileTrue(m_armSubsystem.set(0.3));
    new JoystickButton(m_controller, 5).whileTrue(m_armSubsystem.set(-0.3));

    //Run a sysID routine while Button 6 is held, cancels on release
    new JoystickButton(m_controller, 6).whileTrue(m_armSubsystem.sysId());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
