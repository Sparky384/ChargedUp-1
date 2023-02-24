package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slider extends SubsystemBase{
    private CANSparkMax motorOne; 
    private CANSparkMax motorTwo; 
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_pidController;

    public Slider(){
        motorOne = new CANSparkMax(Constants.CANPorts.slider, MotorType.kBrushless);
        //motorTwo = new CANSparkMax(Constants.CANPorts.elevatorRight, MotorType.kBrushless); 

        m_pidController = motorOne.getPIDController();
        m_pidController.setP(Constants.PIDValues.elevatorOneP);
        m_pidController.setI(Constants.PIDValues.elevatorOneI);
        m_pidController.setD(Constants.PIDValues.elevatorOneD);
        m_pidController.setOutputRange(-0.1, 0.1);
        // initialize SPARK MAX with CAN ID
        m_encoder = motorOne.getEncoder();
        m_pidController.setFeedbackDevice(m_encoder);
        m_encoder.setPosition(0);
        //m_encoder.setInverted(true);
    }

    public void stop(){ 
        motorOne.stopMotor();
    }

    public void move(double distance){
        distance = distance; //replace this with conversion function
        motorOne.getPIDController().setReference(distance, CANSparkMax.ControlType.kPosition);
        //motorTwo.set(-motorOne.get());
    }

    public double getDistance() {
        double distance = motorOne.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
        distance = distance; //replace this with conversion function
        return distance;
    }
    public void periodic() {
        SmartDashboard.putNumber("distance", getDistance());
    }
}
