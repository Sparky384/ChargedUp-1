package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{
    private CANSparkMax motor; 

    public Wrist(){
        motor = new CANSparkMax(Constants.CANPorts.wrist, MotorType.kBrushless); 
        motor.getPIDController().setP(Constants.PIDValues.wristP);
        motor.getPIDController().setI(Constants.PIDValues.wristI); 
        motor.getPIDController().setD(Constants.PIDValues.wristD); 
    }

    public void stop(){
        motor.stopMotor();
    }

    public void driveToAngle(double angle){
        angle = angle; //replace this with conversion function
        motor.getPIDController().setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public double getAngle() 
    {
        double angle = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
        angle = angle; //replace this with conversion function

        return angle;
    }
}
