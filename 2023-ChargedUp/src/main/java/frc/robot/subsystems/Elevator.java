package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//The method addRequirements(Subsystem...) in the type CommandBase is not applicable for the arguments (Elevator)Java(67108979)
public class Elevator extends SubsystemBase {
    private WPI_TalonFX motorOne; 
    private WPI_TalonFX motorTwo; 
    private CANCoder wristEncoder;
    private WPI_TalonFX wristMotor;

    //private Timer pidTimer;


    public Elevator(){
        motorOne = new WPI_TalonFX(Constants.CANPorts.elevatorLeft);
        motorTwo = new WPI_TalonFX(Constants.CANPorts.elevatorRight);
        wristEncoder = new CANCoder(Constants.CANPorts.wristCancoder);
        wristMotor = new WPI_TalonFX(Constants.CANPorts.wristMotor);
        //pidTimer = new Timer();

        motorTwo.follow(motorOne);
        motorOne.setSelectedSensorPosition(0.0);
        motorOne.setNeutralMode(NeutralMode.Brake);
        motorTwo.setNeutralMode(NeutralMode.Brake);
        
        /* Motion Magic Configs for Elevator Motor */
        motorOne.configForwardSoftLimitEnable(true);
        motorOne.configForwardSoftLimitThreshold(Constants.Subsys.elevatorUpperLimit);
        motorOne.configReverseSoftLimitEnable(true);
        motorOne.configReverseSoftLimitThreshold(Constants.Subsys.elevatorLowerLimit);
        motorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        motorOne.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
        motorOne.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
        motorOne.configNominalOutputForward(0, 10);
        motorOne.configNominalOutputReverse(0, 10);
        motorOne.configPeakOutputForward(1, 10);
        motorOne.configPeakOutputReverse(-1, 10);
        motorOne.configMotionAcceleration(6400, 0);
        motorOne.configMotionCruiseVelocity(6400, 0);
        
        // initialize SPARK MAX with CAN ID
        motorOne.config_kP(0, 0.1047); //
        motorOne.config_kI(0, 0.0);
        motorOne.config_kD(0, 0.0);
        motorOne.config_kF(0, 0.05863);
        motorOne.configAllowableClosedloopError(0, 0);
        /*
        // slots for going down the elevator.
        motorOne.config_kP(1, Constants.PIDValues.elevatorOnePDown);
        motorOne.config_kP(1, Constants.PIDValues.elevatorOneI);
        motorOne.config_kP(1, Constants.PIDValues.elevatorOneD);
        motorOne.config_kF(1, Constants.PIDValues.elevatorOneF);
        motorOne.configClosedLoopPeakOutput(0, 0.45); */
        //m_encoder.setInverted(true);

        /*PIDs for wrist motor */
        wristMotor.config_kP(0, 0.0);
        wristMotor.config_kI(0, 0.0);
        wristMotor.config_kD(0, 0.0);
        wristMotor.config_kF(0, 0.0);

        /* Motion Magic Configs for Wrist */
        wristMotor.configClosedLoopPeakOutput(0, 0.1);
        wristMotor.configForwardSoftLimitEnable(true);
        wristMotor.configForwardSoftLimitThreshold(Constants.Subsys.wristUpperLimit); //800
        wristMotor.configReverseSoftLimitEnable(true);
        wristMotor.configReverseSoftLimitThreshold(Constants.Subsys.wristLowerLimit); //-400
        wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
        wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
        wristMotor.configNominalOutputForward(0, 10);
        wristMotor.configNominalOutputReverse(0, 10);
        wristMotor.configPeakOutputForward(1, 10);
        wristMotor.configPeakOutputReverse(-1, 10);
        wristMotor.configMotionAcceleration(6400, 0);
        wristMotor.configMotionCruiseVelocity(6400, 0);
        
    }

    public void stop(){
        motorOne.set(ControlMode.PercentOutput, 0.0); 
    }

    public CommandBase elevatorMotionMagic(double finalPosition){
        return runOnce(() -> motorOne.set(TalonFXControlMode.MotionMagic, finalPosition, DemandType.ArbitraryFeedForward, 0.06687))
        .andThen(Commands.waitUntil(() -> motorOne.getActiveTrajectoryPosition() < finalPosition + 1000 
        && motorOne.getActiveTrajectoryPosition() > finalPosition - 1000).withTimeout(1.5))
        .andThen(runOnce(() -> motorOne.set(TalonFXControlMode.PercentOutput, .06687)));
    }

    public CommandBase wristMotionMagic(double finalPosition) {
        return runOnce(() -> wristMotor.set(TalonFXControlMode.MotionMagic, finalPosition, DemandType.ArbitraryFeedForward, 0.06687))
        .andThen(Commands.waitUntil(() -> wristMotor.getActiveTrajectoryPosition() < finalPosition + 1000 
        && wristMotor.getActiveTrajectoryPosition() > finalPosition - 1000).withTimeout(1.5))
        .andThen(runOnce(() -> wristMotor.set(TalonFXControlMode.PercentOutput, .06687)));
    }

    
    public CommandBase wristUp() {
        return run(() -> wristMotor.set(TalonFXControlMode.PercentOutput, 0.5)).
        finallyDo(interrupted -> wristMotor.set(TalonFXControlMode.PercentOutput, 0.06687)).
        withName("driveWrist");
    }

    public CommandBase wristDown() {
        return run(() -> wristMotor.set(TalonFXControlMode.PercentOutput, -0.5)).
        finallyDo(interrupted -> wristMotor.set(TalonFXControlMode.PercentOutput, 0.06687)).
        withName("driveWrist");
    }

    
    public void move(double height){
        //height /= Constants.ConversionValues.elevatorConversionFunction; //uses encoder counts.
        double difference = height - getHeight(); 
        double output = difference * Constants.PIDValues.elevatorOneP;
        output += Constants.PIDValues.elevatorOneF; 
        if (output > Constants.PIDValues.elevatorMaxSpeed)
            output = Constants.PIDValues.elevatorMaxSpeed;
        else if (output < Constants.PIDValues.elevatorMinSpeed)
            output = Constants.PIDValues.elevatorMinSpeed;
        motorOne.set(ControlMode.PercentOutput, output);
    }

    public CommandBase drive()
    {
        return run(() -> motorOne.set(TalonFXControlMode.PercentOutput, 0.7)).
        finallyDo(interrupted -> motorOne.set(TalonFXControlMode.PercentOutput, 0.06687)).
        withName("driveElevator");
        //motorOne.set(ControlMode.PercentOutput, speed);
    }

    public CommandBase driveDown()
    {
        return run(() -> motorOne.set(TalonFXControlMode.PercentOutput, -0.2)).
        finallyDo(interrupted -> motorOne.set(TalonFXControlMode.PercentOutput, 0.06687)).
        withName("driveElevator");
        //motorOne.set(ControlMode.PercentOutput, speed);
    }

    public double getHeight() {
        double height = motorOne.getSelectedSensorPosition();// was multiplying get position by negative 1
        height *= Constants.ConversionValues.elevatorConversionFunction; //uses inches (to display on screen)
        return height;
    }

    public void periodic() {
        SmartDashboard.putNumber("height", getHeight());
        //SmartDashboard.putNumber("elevatorPosition", m_encoder.getPosition());
        SmartDashboard.putNumber("elevatorSpeed", getHeight());
    }
}

