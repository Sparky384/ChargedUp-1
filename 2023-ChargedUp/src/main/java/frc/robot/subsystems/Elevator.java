package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.CANPorts;
import frc.robot.Constants;

public class Elevator {
    private CANSparkMax motorOne; 
    private CANSparkMax motorTwo; 

    public Elevator(){
        motorOne = new CANSparkMax(CANPorts.elevatorOne, MotorType.kBrushless);
        motorTwo = new CANSparkMax(CANPorts.elevatorTwo, MotorType.kBrushless); 
        motorTwo.follow(motorOne);
        motorOne.getPIDController().setP(Constants.PIDValues.elevatorOneP); 
        motorOne.getPIDController().setI(Constants.PIDValues.elevatorOneI);
        motorOne.getPIDController().setD(Constants.PIDValues.elevatorOneD);
    }

    public void stop(){
        motorOne.stopMotor(); 
    }

    public void move(double distance){
        distance = distance; //replace this with conversion function
        motorOne.getPIDController().setReference(distance, CANSparkMax.ControlType.kPosition); 
    }
}
