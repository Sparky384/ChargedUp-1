package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slider extends SubsystemBase{
    private CANSparkMax motor;

    public Slider(){
        motor = new CANSparkMax(Constants.CANPorts.slider, MotorType.kBrushless);
        motor.getPIDController().setP(Constants.PIDValues.sliderP); 
        motor.getPIDController().setI(Constants.PIDValues.sliderI);
        motor.getPIDController().setD(Constants.PIDValues.sliderD);
    }

    public void stop(){ 
        motor.stopMotor();
    }

    public void move(double distance){
        distance = distance; //replace this with conversion function
        motor.getPIDController().setReference(distance, CANSparkMax.ControlType.kPosition);
    }

    public double getDistance() {
        double distance = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
        distance = distance; //replace this with conversion function

        return distance;
    }
}
