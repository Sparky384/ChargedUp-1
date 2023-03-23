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
import frc.robot.commands.DriveOverRamp;
import frc.robot.commands.GyroStabalize;
import frc.robot.commands.Pause;
import frc.robot.commands.Stow;
import frc.robot.commands.ToHigh;
import frc.robot.paths.JustRamp;
import frc.robot.paths.JustRampNew;
import frc.robot.paths.JustRampNew2;
import frc.robot.subsystems.*;

public class Score1RampNew extends SequentialCommandGroup {
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
            //ToHigh.getToHigh(s_Elevator, s_Slider, wrist),
            //new OuttakeAuto(s_Hand), //outtake for cone.
            //Stow.getStowCommand(s_Elevator, s_Slider, wrist),
            JustRampNew.followTrajectoryCommand(true, s_Swerve),
            JustRampNew2.followTrajectoryCommand(true, s_Swerve),
            new GyroStabalize(s_Swerve)
        );
    }
}
