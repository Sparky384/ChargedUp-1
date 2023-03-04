package frc.robot.paths.pathGroups.leftGroups;
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
import frc.robot.paths.leftPaths.LScore2nd;
import frc.robot.commands.ElevatorFunctionality.MoveElevator;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.WristFunctionality.*;

public class LScore2 extends SequentialCommandGroup {
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
        /* after this call: elevator is low and we're on the next object's point. */
        LScore1.followTrajectoryCommand(s_Swerve, s_Elevator, s_Slider, s_Wrist, s_Hand),
        
        /* Prime piece for intaking. put limelight here if using it. */
        new ParallelCommandGroup(new MoveSlider(s_Slider, Constants.Subsys.sliderOut), new RotateWrist(s_Wrist, Constants.Subsys.wristLow)),

        /* intake new piece */
        new IntakeAuto(s_Hand),

        /* Move to goal for 2nd score and put elevator up in preperation. */
        new ParallelCommandGroup(new MoveElevator(s_Elevator, Constants.Subsys.elevatorHigh), LScore2nd.followTrajectoryCommand(false, s_Swerve)),
        
        /* Score 2nd piece */
        new OuttakeAuto(s_Hand),

        /* Intakes slider and wrist into robot. */
        new ParallelCommandGroup(new MoveSlider(s_Slider, Constants.Subsys.sliderIn),new RotateWrist(s_Wrist, Constants.Subsys.wristHigh))
        /* DO NOT MOVE TO NEXT PIECE AS RAMP AUTO CAN BE CALLED AFTER THIS */
        );
    }
}
