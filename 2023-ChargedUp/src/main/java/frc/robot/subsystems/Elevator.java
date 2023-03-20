package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private boolean wristMoving;

    //private Timer pidTimer;


    public Elevator(){
        motorOne = new WPI_TalonFX(Constants.CANPorts.elevatorLeft);
        motorTwo = new WPI_TalonFX(Constants.CANPorts.elevatorRight);
        wristEncoder = new CANCoder(Constants.CANPorts.wristCancoder);
        wristMotor = new WPI_TalonFX(Constants.CANPorts.wristMotor);
        //pidTimer = new Timer();
        wristMoving = false;

        wristMotor.setSensorPhase(true);

        motorTwo.follow(motorOne);
        motorOne.setSelectedSensorPosition(0.0);
        motorOne.setNeutralMode(NeutralMode.Brake); //was on brake
        motorTwo.setNeutralMode(NeutralMode.Brake); //was on brake
        
        /* Motion Magic Configs for Elevator Motor */
        motorOne.configForwardSoftLimitEnable(true);
        motorOne.configForwardSoftLimitThreshold(Constants.Subsys.elevatorUpperLimit);
        motorOne.configReverseSoftLimitEnable(true);
        motorOne.configReverseSoftLimitThreshold(Constants.Subsys.elevatorLowerLimit);
        
        motorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        motorOne.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.Subsys.timeOutMs);
        motorOne.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.Subsys.timeOutMs);
        
        motorOne.configNominalOutputForward(0, Constants.Subsys.timeOutMs);
        motorOne.configNominalOutputReverse(0, Constants.Subsys.timeOutMs);
        motorOne.configPeakOutputForward(1, Constants.Subsys.timeOutMs);
        motorOne.configPeakOutputReverse(-1, Constants.Subsys.timeOutMs);
        
        motorOne.configMotionAcceleration(6400, 0);
        motorOne.configMotionCruiseVelocity(6400, 0);
        
        /* Slot 0 for going up the elevator */
        motorOne.config_kP(0, Constants.PIDValues.elevatorUpP); //0.1047
        motorOne.config_kI(0, Constants.PIDValues.elevatorUpI);
        motorOne.config_kD(0, Constants.PIDValues.elevatorUpD);
        motorOne.config_kF(0, Constants.PIDValues.elevatorUpF); //0.05863
        motorOne.configAllowableClosedloopError(0, 0);
        
        /*Slow 1 for going down elevator */
        motorOne.config_kP(1, Constants.PIDValues.elevatorDownP); //0.1047
        motorOne.config_kI(1, Constants.PIDValues.elevatorDownI);
        motorOne.config_kD(1, Constants.PIDValues.elevatorDownD);
        motorOne.configAllowableClosedloopError(1, 0);
        //motorOne.configClosedLoopPeakOutput(1, 0.45);
        /*
        // Slot 1 for going down the elevator.
         */

        /*PIDs for wrist motor */
        wristMotor.config_kP(0, 1.0);
        wristMotor.config_kI(0, 0.0);
        wristMotor.config_kD(0, 2.5);
        wristMotor.config_kF(0, 0.0);
        
        wristMotor.set(ControlMode.PercentOutput, 0.0);
        wristMotor.setInverted(true);
        wristMotor.setNeutralMode(NeutralMode.Brake);

        /* Motion Magic Configs for Wrist */
        wristMotor.configClosedLoopPeakOutput(0, 0.1);
        wristMotor.configForwardSoftLimitEnable(true);
        wristMotor.configForwardSoftLimitThreshold(Constants.Subsys.wristLowerLimit); //800 119
        wristMotor.configReverseSoftLimitEnable(true);
        wristMotor.configReverseSoftLimitThreshold(Constants.Subsys.wristUpperLimit); //-400 1534
        
        wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.Subsys.timeOutMs);
        wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.Subsys.timeOutMs);
        
        wristMotor.configNominalOutputForward(0, Constants.Subsys.timeOutMs);
        wristMotor.configNominalOutputReverse(0, Constants.Subsys.timeOutMs);
        wristMotor.configPeakOutputForward(1, Constants.Subsys.timeOutMs);
        wristMotor.configPeakOutputReverse(-1, Constants.Subsys.timeOutMs);
        wristMotor.configMotionAcceleration(3500, 0);
        wristMotor.configMotionCruiseVelocity(3500, 0);
        
    }

    public void stop(){
        motorOne.set(ControlMode.PercentOutput, 0.0); 
    }

    public CommandBase elevatorMotionMagic(double finalPosition)
    {
        /* if true we're going down if false we're going up. */
        double feed;
        if (finalPosition < motorOne.getSelectedSensorPosition()) 
        {
            feed = Constants.Subsys.elevatorArbitraryFeedForward * 0;
            motorOne.selectProfileSlot(1, 0);
            motorOne.set(ControlMode.PercentOutput, 0.0); //was 0.0
            return runOnce(() -> {});
        }
        else
        {
            motorOne.setNeutralMode(NeutralMode.Brake);
            feed = Constants.Subsys.elevatorArbitraryFeedForward;
            motorOne.selectProfileSlot(0, 0);
            return runOnce(() -> motorOne.set(TalonFXControlMode.MotionMagic, finalPosition, DemandType.ArbitraryFeedForward, feed))
            .andThen(Commands.waitUntil(() -> motorOne.getActiveTrajectoryPosition() < finalPosition + Constants.Subsys.elevatorThreshold 
            && motorOne.getActiveTrajectoryPosition() > finalPosition - Constants.Subsys.elevatorThreshold).withTimeout(3))
            .andThen(runOnce(() -> motorOne.set(TalonFXControlMode.PercentOutput, Constants.Subsys.elevatorArbitraryFeedForward)));
        } 
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
        
    }
}

