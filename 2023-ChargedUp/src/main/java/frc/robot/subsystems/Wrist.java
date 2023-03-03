package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{
    private CANSparkMax motor; 
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_pidController;

    public Wrist(){
        motor = new CANSparkMax(Constants.CANPorts.wrist, MotorType.kBrushless); 
        m_pidController = motor.getPIDController();
        m_pidController.setP(Constants.PIDValues.wristP);
        m_pidController.setI(Constants.PIDValues.wristI); 
        m_pidController.setD(Constants.PIDValues.wristD); 
        m_pidController.setOutputRange(-0.1, 0.1);


        m_encoder = motor.getEncoder();
        m_pidController.setFeedbackDevice(m_encoder);
        m_encoder.setPosition(0);
    }

    public void stop(){
        motor.stopMotor();
    }

    public void driveToAngle(double angle){
        angle = angle; //replace this with conversion function
        m_pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public double getAngle() 
    {
        double angle = m_encoder.getPosition();
        angle = angle; //replace this with conversion function
        return angle;
    }
}
