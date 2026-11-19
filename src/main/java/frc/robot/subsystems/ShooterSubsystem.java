package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;


public class ShooterSubsystem extends SubsystemBase {

  // What the motor controller should do: its control mode, gains and gearing.
  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback constants (PID), on the real robot and in simulation.
      .withClosedLoopController(.0015, 0, 0)
      .withSimClosedLoopController(1, 0, 0)
      // Feedforward constants, on the real robot and in simulation.
      .withFeedforward(new SimpleMotorFeedforward(.15, .125, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      // Gearing from the motor rotor to the flywheel: 12:1 in total.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      // Coast, so that the wheel spins down by itself.
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      // Telemetry name and how much to publish.
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH);

  // The vendor motor controller object: a SPARK MAX with CAN ID 4.
  private final SparkMax spark = new SparkMax(17, MotorType.kBrushless);

  // The SmartMotorController applies the config to the SPARK MAX, which drives one NEO.
  private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig()
      // The diameter of the flywheel, used by the simulation and for surface speeds.
      .withDiameter(Inches.of(4))
      // Telemetry name and how much to publish.
      .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  // The flywheel mechanism.
  private final FlyWheel shooter = new FlyWheel(shooterConfig, motor);

  /** Spin at a speed and hold it. This command does not end on its own. */
  public Command run(AngularVelocity speed) {
    return shooter.run(speed);
  }

  /** Drive the flywheel with a duty cycle from -1 to 1. The closed loop is not used. */
  public Command set(double dutycycle) {
    return shooter.set(dutycycle);
  }

  /** True while the flywheel is within 50 RPM of a speed. */
  public Trigger atSpeed(AngularVelocity speed) {
    return shooter.isNear(speed, RPM.of(50));
  }

  @Override
  public void periodic() {
    // Publishes the flywheel's telemetry, and applies anything changed through live tuning.
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Runs the flywheel's physics simulation.
    shooter.simIterate();
  }
}
