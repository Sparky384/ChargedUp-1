package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.Drive;
import frc.robot.commands.Pause;
import frc.robot.commands.Stow;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToGround;
import frc.robot.commands.ToHigh;
import frc.robot.paths.rightPaths.*;
import frc.robot.commands.ElevatorFunctionality.MoveElevator;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.WristFunctionality.*;
import frc.robot.commands.Pause;

import frc.robot.paths.rightPaths.Blue2ScoreRightPickup;
import frc.robot.paths.rightPaths.Blue2ScoreRightScore;
import frc.robot.paths.rightPaths.Blue2ScoreRightRamp;

public class DistrictTestAuto extends SequentialCommandGroup{
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
        Blue2ScoreRightScore.followTrajectoryCommand(false, s_Swerve)
        );
    }
}
