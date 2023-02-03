package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.CANPorts;
import frc.robot.Constants;

public class Slider {
    private CANSparkMax motor;

    public Slider(){
        motor = new CANSparkMax(CANPorts.slider, MotorType.kBrushless);
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
}
