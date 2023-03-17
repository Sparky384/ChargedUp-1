package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slider extends SubsystemBase{
    private CANSparkMax motorOne; 
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_pidController;
    private double sliderRawCounts;

    public Slider(){
        motorOne = new CANSparkMax(Constants.CANPorts.slider, MotorType.kBrushless);
        motorOne.setIdleMode(IdleMode.kBrake);
        //motorTwo = new CANSparkMax(Constants.CANPorts.elevatorRight, MotorType.kBrushless); 

        m_pidController = motorOne.getPIDController();
        m_pidController.setP(Constants.PIDValues.sliderP);
        m_pidController.setI(Constants.PIDValues.sliderI);
        m_pidController.setIZone(Constants.PIDValues.sliderIZone);
        m_pidController.setIMaxAccum(0.2, 0);
        m_pidController.setD(Constants.PIDValues.sliderD);
        m_pidController.setOutputRange(-0.35, 0.35);
        // initialize SPARK MAX with CAN ID
        m_encoder = motorOne.getEncoder();
        m_pidController.setFeedbackDevice(m_encoder);
        m_encoder.setPosition(0);
        //m_encoder.setInverted(true);
    }

    public void stop(){ 
        motorOne.stopMotor();
    }

    public void toggle()
    {
        if (motorOne.getIdleMode() == IdleMode.kBrake)
            motorOne.setIdleMode(IdleMode.kCoast);
        else
            motorOne.setIdleMode(IdleMode.kBrake);
    }

    public void move(double distance){
        distance /= Constants.ConversionValues.sliderConversionFunction; //uses encoder counts.
        m_pidController.setReference(distance, CANSparkMax.ControlType.kPosition);

    }

    public double getDistance() {
        double distance = m_encoder.getPosition();
        sliderRawCounts = distance; //testing purposes - remove later.
        distance *= Constants.ConversionValues.sliderConversionFunction; //uses inches (to display)
        return distance;
    }
    public void periodic() {
        SmartDashboard.putNumber("s_sliderDistance", getDistance());
        SmartDashboard.putNumber("sliderRawCounts", sliderRawCounts);
    }
}
