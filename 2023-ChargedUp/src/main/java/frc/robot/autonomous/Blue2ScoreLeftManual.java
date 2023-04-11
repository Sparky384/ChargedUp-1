package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.commands.TurnAndMove;
import frc.robot.commands.CommandGroups.Stow;
import frc.robot.commands.CommandGroups.ToGround;
import frc.robot.commands.CommandGroups.ToGroundAutoCube;
import frc.robot.commands.CommandGroups.ToMid;
import frc.robot.commands.DriveFunctionality.Drive;
import frc.robot.commands.DriveFunctionality.DriveToPosition;
import frc.robot.commands.DriveFunctionality.TurnTo;
import frc.robot.commands.WristFunctionality.*;

public class Blue2ScoreLeftManual extends SequentialCommandGroup{
    private static Swerve s_Swerve;
    private static Hand s_Hand;

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve swerve, Elevator elevator, Slider slider, Wrist wrist, Hand hand) {
        s_Swerve = swerve;
        s_Hand = hand;

        return new SequentialCommandGroup(
            new IntakeAuto(hand),
            new DriveToPosition(s_Swerve, new Pose2d(new Translation2d(-43.307, -13.38), new Rotation2d(0)), 1.15),
            new DriveToPosition(s_Swerve, new Pose2d(new Translation2d(-111.417, -13.38), new Rotation2d(0)), 1.6),
            new ParallelCommandGroup(
                new TurnAndMove(s_Swerve, new Pose2d(new Translation2d(-163.384, 4.33), new Rotation2d(0)), 0, 2.15), // 2.4
                ToGroundAutoCube.getToGround(elevator, slider, wrist)),
            new ParallelRaceGroup(
                new DriveToPosition(s_Swerve, new Pose2d(new Translation2d(-214.133, -4.33), new Rotation2d(0)), 1.4),
                new Outtake(hand)
            ),
            Stow.getStowCommand(elevator, slider, wrist),
            new TurnAndMove(s_Swerve, new Pose2d(new Translation2d(-163.384, -4.33), new Rotation2d(0)), -179, 2.25),
            new DriveToPosition(s_Swerve, new Pose2d(new Translation2d(-111.417, -13.38), new Rotation2d(0)), 1.7),
            new DriveToPosition(s_Swerve, new Pose2d(new Translation2d(-43.307, -13.38), new Rotation2d(0)), 1.15),
            new ParallelCommandGroup(
                ToMid.getToMid(elevator, slider, wrist),
                new DriveToPosition(s_Swerve, new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), 1.1)
            ),
            new IntakeAuto(hand)
            );
    }
}