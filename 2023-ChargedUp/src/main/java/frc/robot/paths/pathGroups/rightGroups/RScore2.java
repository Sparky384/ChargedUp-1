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
import frc.robot.paths.rightPaths.RScore2nd;
import frc.robot.paths.rightPaths.RScore3rd;
import frc.robot.commands.Stow;
import frc.robot.commands.ToGround;
import frc.robot.commands.ToHigh;
import frc.robot.commands.ElevatorFunctionality.MoveElevator;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.WristFunctionality.*;

public class RScore2 extends SequentialCommandGroup {
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
            ToHigh.getToHigh(s_Elevator, s_Slider, wrist),
            new OuttakeAuto(s_Hand),
            RPickup2nd.followTrajectoryCommand(true, s_Swerve), //delete later
            ToGround.getToGround(s_Elevator, s_Slider, wrist),
            new IntakeAuto(hand),
            Stow.getStowCommand(s_Elevator, s_Slider, wrist),
            RScore2nd.followTrajectoryCommand(false, s_Swerve),
            ToHigh.getToHigh(s_Elevator, s_Slider, wrist),
            new OuttakeAuto(s_Hand)
        );
    }
}
