// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.TestMotorSubsystem;

public class RobotContainer {
    private final TestMotorSubsystem m_testMotorSubsystem;

    private final GenericHID m_controller =
      new GenericHID(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {
    m_testMotorSubsystem = new TestMotorSubsystem();
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_controller, 1).onTrue(new RunCommand(() -> m_testMotorSubsystem.testMotorMove()));
    new JoystickButton(m_controller, 2).onTrue(new RunCommand(() -> m_testMotorSubsystem.testMotorStop()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
