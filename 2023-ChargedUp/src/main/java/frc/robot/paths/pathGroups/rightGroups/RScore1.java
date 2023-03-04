package frc.robot.paths.pathGroups.rightGroups;

import frc.lib.pathplanner.com.pathplanner.lib.PathPlanner;
import frc.lib.pathplanner.com.pathplanner.lib.PathPlannerTrajectory;
import frc.lib.pathplanner.com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.lib.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.robot.Constants;
import frc.robot.paths.rightPaths.RPickup2nd;
import frc.robot.commands.ElevatorFunctionality.MoveElevator;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.WristFunctionality.*;

public class RScore1 extends SequentialCommandGroup{
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

        /* move elevator up to reach the first pole/cube. */
        new MoveElevator(s_Elevator, Constants.Subsys.elevatorHigh), //may already have elevator to height. otherwise bring up elevator.
        
        /* deploy slider and set wrist down for initial piece scoring */
        new ParallelCommandGroup(new MoveSlider(s_Slider, Constants.Subsys.sliderOut), new RotateWrist(s_Wrist, Constants.Subsys.wristLow)),
        
        /* Score Piece */
        new OuttakeAuto(s_Hand),
        
        /* Move slider and wrist back into robot. */
        new ParallelCommandGroup(new MoveSlider(s_Slider, Constants.Subsys.sliderIn), new RotateWrist(s_Wrist, Constants.Subsys.wristHigh)),

        /* drive and put elevator down to get ready for next object pickup. */
        new ParallelCommandGroup(new MoveElevator(s_Elevator, Constants.Subsys.elevatorLow), RPickup2nd.followTrajectoryCommand(true, s_Swerve)) 
        );
    }
}
