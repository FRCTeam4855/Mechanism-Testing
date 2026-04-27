package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.motorcontrollers.SmartMotorController;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ArmSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(SmartDashboard.getNumber("kP", 50), SmartDashboard.getNumber("kI", 0), SmartDashboard.getNumber("kD", 0), DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withSimClosedLoopController(SmartDashboard.getNumber("Sim kP", 50), SmartDashboard.getNumber("Sim kI", 0), SmartDashboard.getNumber("Sim kD", 0), DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  // Feedforward Constants
  .withFeedforward(new ArmFeedforward(SmartDashboard.getNumber("kS", 0), SmartDashboard.getNumber("kG", 0), SmartDashboard.getNumber("kV", 0)))
  .withSimFeedforward(new ArmFeedforward(SmartDashboard.getNumber("Sim kS", 0), SmartDashboard.getNumber("Sim kG", 0), SmartDashboard.getNumber("Sim kV", 0)))
  // Telemetry name and verbosity level
  .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  // You could also use .withGearing(12) which does the same thing.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(SmartDashboard.getNumber("Motor Reduction", 1))))
  // Motor properties to prevent over currenting.
  .withMotorInverted(SmartDashboard.getBoolean("Motor Inverted", false))
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(11, MotorType.kBrushless);
  
  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);  

  private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
  // Soft limit is applied to the SmartMotorControllers PID
  .withSoftLimits(Degrees.of(0), Degrees.of(45))
  // Hard limit is applied to the simulation.
  .withHardLimit(Degrees.of(0), Degrees.of(90))
  // Starting position is where your arm starts
  .withStartingPosition(Degrees.of(90))
  // Length and mass of your arm for sim.
  .withLength(Feet.of(3))
  .withMass(Pounds.of(1))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  // Arm Mechanism
  private Arm arm = new Arm(armCfg);  

	  /**
   * Set the angle of the arm, does not stop when the arm reaches the setpoint.
   * @param angle Angle to go to.
   * @return A command.
   */
  public Command setAngle(Angle angle) { return arm.run(angle);}
  
  /**
   * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the setpoint.
   * @param angle Angle to go to.
   * @param tolerance Angle tolerance for completion.
   * @return A Command
   */
  public Command setAngleAndStop(Angle angle, Angle tolerance) { return arm.runTo(angle, tolerance);}
  
  /**
   * Set arm closed loop controller to go to the specified mechanism position.
   * @param angle Angle to go to.
   */
  public void setAngleSetpoint(Angle angle) { arm.setMechanismPositionSetpoint(angle); }

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) { return arm.set(dutycycle);}

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() { return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}


  public ArmSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }  
    
}
