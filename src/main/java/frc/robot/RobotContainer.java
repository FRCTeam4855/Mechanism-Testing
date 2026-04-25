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
import frc.robot.subsystems.FlywheelSubsystem;
import static edu.wpi.first.units.Units.RPM;

public class RobotContainer {
    private final FlywheelSubsystem m_flywheelSubsystem;

    private final GenericHID m_controller =
      new GenericHID(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {
    m_flywheelSubsystem = new FlywheelSubsystem();
    configureBindings();
    // Set the default command to force the shooter rest.
    m_flywheelSubsystem.setDefaultCommand(m_flywheelSubsystem.set(0));
  }

  private void configureBindings() {
    //buttons for preset RPM setpoints
    new JoystickButton(m_controller, 1).onTrue(m_flywheelSubsystem.setVelocity(RPM.of(500)).repeatedly());
    new JoystickButton(m_controller, 2).onTrue(m_flywheelSubsystem.setVelocity(RPM.of(0)).repeatedly());

    new JoystickButton(m_controller, 6).onTrue(new RunCommand(()-> m_flywheelSubsystem.setVelocity(RPM.of(500))));
    new JoystickButton(m_controller, 8).onTrue(new RunCommand(()-> m_flywheelSubsystem.setVelocity(RPM.of(0))));

    //button for customizable RPM setpoint from SmartDashboard
    new JoystickButton(m_controller, 5).onTrue(m_flywheelSubsystem.setVelocity(RPM.of(SmartDashboard.getNumber("Flywheel Setpoint (RPM)", 0))));
    new JoystickButton(m_controller, 10).onTrue(new RunCommand(()-> m_flywheelSubsystem.setVelocity(RPM.of(SmartDashboard.getNumber("Flywheel Setpoint (RPM)", 0)))));

    //buttons for adjusting flywheel speed (upping and lowering but 0.3 or -0.3)
    new JoystickButton(m_controller,3).onTrue(m_flywheelSubsystem.set(0.3));
    new JoystickButton(m_controller, 4).onTrue(m_flywheelSubsystem.set(-0.3));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
