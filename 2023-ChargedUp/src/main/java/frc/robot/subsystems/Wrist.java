package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{
    private WPI_TalonFX motor; 
    private CANCoder m_encoder;

    public Wrist(){
        
        motor = new WPI_TalonFX(Constants.CANPorts.wristMotor);
        m_encoder = new CANCoder(Constants.CANPorts.wristCancoder);
        /*
        motor.config_kP(0, Constants.PIDValues.elevatorOneP);
        motor.config_kP(0, Constants.PIDValues.elevatorOneI);
        motor.config_kP(0, Constants.PIDValues.elevatorOneD);
        motor.configClosedLoopPeakOutput(0, 0.1); //speed being limited to .3 of max

        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);    
        //m_encoder.setPosition(Constants.Subsys.wristAbsEncoderOffset);
        //motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 100);
        */
    }

    public void stop(){
        motor.stopMotor();
    }

    public void driveToAngle(double angle){
        angle /= Constants.ConversionValues.wristConversionFunction; //uses encoder counts - could be switched to .00630366 without /= just =
        SmartDashboard.putNumber("driveToWristAngle", angle);
        //motor.set(ControlMode.PercentOutput, angle);
    }

    public double getAngle() 
    {
        return m_encoder.getAbsolutePosition();
    }

    public void periodic() {
        SmartDashboard.putNumber("wristAngle", getAngle());
        //SmartDashboard.putNumber("elevatorPosition", m_encoder.getPosition());
        SmartDashboard.putNumber("wristSensorPosition", motor.getSelectedSensorPosition());
    }
}
