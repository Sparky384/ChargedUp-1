package frc.robot.paths.pathGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.pathplanner.com.pathplanner.lib.PathPlanner;
import frc.lib.pathplanner.com.pathplanner.lib.PathPlannerTrajectory;
import frc.lib.pathplanner.com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.Swerve;
import frc.lib.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.robot.Constants;
import frc.robot.commands.ElevatorFunctionality.MoveElevator;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.WristFunctionality.*;
import frc.robot.commands.DriveOnRamp;
import frc.robot.commands.GyroStabalize;
import frc.robot.paths.JustRamp;
import frc.robot.subsystems.*;

public class Score1Ramp extends SequentialCommandGroup {
    private static Swerve s_Swerve;
    private static Elevator s_Elevator;
    private static Slider s_Slider;
    private static Wrist s_Wrist;
    private static Hand s_Hand;

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve swerve, Elevator elevator, Slider slider, Wrist wrist, Hand hand) {
        s_Swerve = swerve;
        s_Elevator = elevator;
        s_Slider = slider;
        s_Wrist = wrist;
        s_Hand = hand;

        return new SequentialCommandGroup(
            //limelight center target - maybe and if so put in parallel command with elevator setup.

            new MoveElevator(s_Elevator, Constants.Subsys.elevatorHigh), //may already have elevator to height. otherwise bring up elevator.
    
            /* deploy slider and set wrist down for initial piece scoring */
            new ParallelCommandGroup(new MoveSlider(s_Slider, Constants.Subsys.sliderOut), new RotateWrist(s_Wrist, Constants.Subsys.wristLow)),
            
            /* Score Piece */
            new OuttakeAuto(s_Hand),
            
            /* Move slider and wrist back into robot. */
            new ParallelCommandGroup(new MoveSlider(s_Slider, Constants.Subsys.sliderIn), new RotateWrist(s_Wrist, Constants.Subsys.wristHigh)),
    
            /* drive and put elevator down to get ready for next object pickup. */
            new ParallelCommandGroup(new MoveElevator(s_Elevator, Constants.Subsys.elevatorLow), JustRamp.followTrajectoryCommand(true, s_Swerve)),

            /* Drive up ramp backward and balance. */
            new DriveOnRamp(s_Swerve, true),
            new GyroStabalize(s_Swerve)
        );
    }
}
