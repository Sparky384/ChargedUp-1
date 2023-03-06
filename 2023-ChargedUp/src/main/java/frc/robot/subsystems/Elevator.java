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
//The method addRequirements(Subsystem...) in the type CommandBase is not applicable for the arguments (Elevator)Java(67108979)
public class Elevator extends SubsystemBase {
    private CANSparkMax motorOne; 
    private CANSparkMax motorTwo; 
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_pidController;


    public Elevator(){
        motorOne = new CANSparkMax(Constants.CANPorts.elevatorLeft, MotorType.kBrushless);
        motorTwo = new CANSparkMax(Constants.CANPorts.elevatorRight, MotorType.kBrushless); 

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

    public void move(double height){
        height = height; //replace this with conversion function
        m_pidController.setReference(height, CANSparkMax.ControlType.kPosition); 
        motorTwo.set(-motorOne.get());
    }

    public double getHeight() {
        double height = m_encoder.getPosition();// was multiplying get position by negative 1
        height = height; //replace this with conversion function
        return height;
    }

    public void periodic() {
        SmartDashboard.putNumber("height", getHeight());
    }
}

