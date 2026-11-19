// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ShooterSubsystem;
import static edu.wpi.first.units.Units.RPM;

public class RobotContainer {
  private final ShooterSubsystem m_flywheelSubsystem;

  private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {
    m_flywheelSubsystem = new ShooterSubsystem();
    configureBindings();
    // Set the default command to force the shooter rest.
    m_flywheelSubsystem.setDefaultCommand(new RunCommand(
        () -> m_flywheelSubsystem.set(0), m_flywheelSubsystem));
  }

  private void configureBindings() {

    m_controller.a().whileTrue(m_flywheelSubsystem.run(RPM.of(1000)));
    m_controller.b().whileTrue(m_flywheelSubsystem.run(RPM.of(0)));
    m_controller.x().whileTrue(m_flywheelSubsystem.set(0.2));
    m_controller.y().whileTrue(m_flywheelSubsystem.set(-0.2));
    m_controller.rightBumper().whileTrue(m_flywheelSubsystem.set(0));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
