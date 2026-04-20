package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotorSubsystem extends SubsystemBase {
  private final SparkMax m_testMotor;
  private final SparkAbsoluteEncoder m_testMotorEncoder;
  private final SparkClosedLoopController m_testMotorController;

    /** Creates a new TestMotorSubsystem. */
  public TestMotorSubsystem() {
    m_testMotor = new SparkMax(Constants.MotorConstants.kTestMotorID, MotorType.kBrushless);
    m_testMotorEncoder = m_testMotor.getAbsoluteEncoder();
    m_testMotorController = m_testMotor.getClosedLoopController();
  }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Test Motor Position", m_testMotorEncoder.getPosition());
    }
    
    public void testMotorMove() {
        m_testMotorController.setSetpoint(SmartDashboard.getNumber("Test Motor Speed", MotorConstants.kTestMotorSpeed), ControlType.kVelocity);
    }

    public void testMotorStop() {
        m_testMotor.set(0);
    }
}
