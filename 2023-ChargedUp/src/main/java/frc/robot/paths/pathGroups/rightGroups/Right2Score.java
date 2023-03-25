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
import frc.robot.commands.Drive;
import frc.robot.commands.DriveTo;
import frc.robot.commands.Stow;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToGround;
import frc.robot.commands.ToHigh;
import frc.robot.commands.TurnTo;
import frc.robot.paths.rightPaths.*;
import frc.robot.commands.ElevatorFunctionality.MoveElevator;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.WristFunctionality.*;

public class Right2Score extends SequentialCommandGroup{
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
            new IntakeAuto(s_Hand), //assumes we start with cube so this is outtake for cube
            new DriveTo(s_Swerve, 2.0, -3.0, 0.4, 0.4),
            new DriveTo(s_Swerve, -150.0, 0.0, 0.4, 0.0),
            new TurnTo(s_Swerve, 15),
            ToGround.getToGround(s_Elevator, s_Slider, s_Wrist),
            new ParallelCommandGroup(new OuttakeAuto(s_Hand, 1.75), new Drive(s_Swerve, 0.22, 0.0, 1.5, 0.0))
        );
    }
}