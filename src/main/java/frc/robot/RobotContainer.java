// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private final ElevatorSubsystem m_elevatorSubsystem;

    private final GenericHID m_controller =
      new GenericHID(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {
    m_elevatorSubsystem = new ElevatorSubsystem();
    // Set the default command to force the elevator to go to 0.
    m_elevatorSubsystem.setDefaultCommand(m_elevatorSubsystem.setHeight(Meters.of(0)));
    configureBindings();
  }

  private void configureBindings() {
    //preset heights
	  new JoystickButton(m_controller, 1).onTrue(m_elevatorSubsystem.setHeight(Meters.of(0)).repeatedly());
    new JoystickButton(m_controller, 2).onTrue(m_elevatorSubsystem.setHeight(Meters.of(1)).repeatedly());

    new JoystickButton(m_controller, 6).onTrue(new RunCommand(()-> m_elevatorSubsystem.setHeight(Meters.of(0))));
    new JoystickButton(m_controller, 8).onTrue(new RunCommand(()-> m_elevatorSubsystem.setHeight(Meters.of(1))));

    new JoystickButton(m_controller, 9).onTrue(new RunCommand(()-> m_elevatorSubsystem.setHeightAndStop(Meters.of(1), Meters.of(0.1))));

    //custom height set from SmartDashboard
    new JoystickButton(m_controller, 5).onTrue(m_elevatorSubsystem.setHeight(Meters.of(SmartDashboard.getNumber("Custom Elevator Height (m)", 0))).repeatedly());
    new JoystickButton(m_controller, 10).onTrue(new RunCommand(()-> m_elevatorSubsystem.setHeightAndStop(Meters.of(SmartDashboard.getNumber("Custom Elevator Height (m)", 0)), Meters.of(0.1))));

    //adjust heights
    new JoystickButton(m_controller, 3).whileTrue(m_elevatorSubsystem.set(0.3));
    new JoystickButton(m_controller, 4).whileTrue(m_elevatorSubsystem.set(-0.3));

    //sysID routine, stops when button is let go
    new JoystickButton(m_controller, 11).whileTrue(m_elevatorSubsystem.sysId());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
