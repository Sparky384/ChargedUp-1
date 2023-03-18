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
import frc.robot.paths.rightPaths.RPickup3rd;
import frc.robot.paths.rightPaths.RScore3rd;
import frc.robot.commands.ElevatorFunctionality.MoveElevator;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.WristFunctionality.*;

// Right Side Of The Arena

public class RScore3 extends SequentialCommandGroup {
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
        /* Call Score2: Elevator is still up after scoring, sliderIn, WristHigh */
        //RScore2.followTrajectoryCommand(s_Swerve, s_Elevator, s_Slider, s_Wrist, s_Hand),
        RScore2.followTrajectoryCommand(s_Swerve, s_Elevator, s_Slider, s_Wrist, s_Hand),
        
        /* Prime elevator for picking up 3rd object and move to 3rd object. */
        //new ParallelCommandGroup(new MoveElevator(s_Elevator, Constants.Subsys.elevatorLow), RPickup3rd.followTrajectoryCommand(false, s_Swerve)),
        RPickup3rd.followTrajectoryCommand(false, s_Swerve), //delete later

        /* Prime wrist and slider for intaking. put limelight here if using it. */
        //new ParallelCommandGroup(new MoveSlider(s_Slider, Constants.Subsys.sliderOut), new RotateWrist(s_Wrist, Constants.Subsys.wristLow)),
        
        /* intake new piece */
        //new IntakeAuto(s_Hand),

        /* Move to goal for 3rd score and put elevator up in preperation. */
        //new ParallelCommandGroup(new MoveElevator(s_Elevator, Constants.Subsys.elevatorHigh), RScore3rd.followTrajectoryCommand(false, s_Swerve)),
        RScore3rd.followTrajectoryCommand(false, s_Swerve) //delete later
        
        /* Take out slider and Wrist. put limelight here if using it. */
        //new ParallelCommandGroup(new MoveSlider(s_Slider, Constants.Subsys.sliderOut), new RotateWrist(s_Wrist, Constants.Subsys.wristLow)),
        
        /* Score 3rd piece */
        //new OuttakeAuto(s_Hand),

        /* Intakes slider and wrist into robot. */
        //new ParallelCommandGroup(new MoveSlider(s_Slider, Constants.Subsys.sliderIn),new RotateWrist(s_Wrist, Constants.Subsys.wristHigh))
        );
    }
}
