package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slider extends SubsystemBase{
    private CANSparkMax motorOne; 
    private RelativeEncoder m_encoder;
    private SparkMaxPIDController m_pidController;

    private REVLibError status = REVLibError.kOk;

    public Slider(){
        motorOne = new CANSparkMax(Constants.CANPorts.slider, MotorType.kBrushless);
        motorOne.setIdleMode(IdleMode.kBrake);
        if (motorOne.getIdleMode() == IdleMode.kBrake)
            System.out.println("Made slider motor " + Constants.CANPorts.slider + " is in mode break");
        else    
            System.out.println("Made slider motor " + Constants.CANPorts.slider + " is in mode coast");

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
    }

    public void stop(){ 
        System.out.println("Slider stop");
        motorOne.stopMotor();
    }

    public void toggle()
    {
        if (motorOne.getIdleMode() == IdleMode.kBrake)
        {
            System.out.println("Toggle to coast");
            motorOne.setIdleMode(IdleMode.kCoast);
        }
        else
        {    
            System.out.println("Toggle to break");
            motorOne.setIdleMode(IdleMode.kBrake);
        }
    }

    public void move(double distance){
        System.out.println("Slider to move to distance " + distance);
        System.out.println("Slider distance in move " + getDistance());
        distance /= Constants.ConversionValues.sliderConversionFunction; //uses encoder counts.
        status = m_pidController.setReference(distance, CANSparkMax.ControlType.kPosition);
    }   

    public double getDistance() {
        double distance = m_encoder.getPosition();
        distance *= Constants.ConversionValues.sliderConversionFunction; //uses inches (to display)
        return distance;
    }
    public void periodic() {
        SmartDashboard.putNumber("slider position", m_encoder.getPosition());
        SmartDashboard.putString("slide rev error status", status.name());
        System.out.println("applied output " +  motorOne.getAppliedOutput());
        System.out.println(status.name()); 
    }
}
