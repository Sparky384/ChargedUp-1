package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.commands.TurnAndMove;
import frc.robot.commands.CommandGroups.Stow;
import frc.robot.commands.CommandGroups.ToGround;
import frc.robot.commands.DriveFunctionality.Drive;
import frc.robot.commands.DriveFunctionality.DriveToPosition;
import frc.robot.commands.DriveFunctionality.TurnTo;
import frc.robot.commands.WristFunctionality.*;

public class TurnAndMoveTest extends SequentialCommandGroup{
    private static Swerve s_Swerve;
    private static Hand s_Hand;

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve swerve, Elevator elevator, Slider slider, Wrist wrist, Hand hand) {
        s_Swerve = swerve;
        s_Hand = hand;

        return new SequentialCommandGroup(
           new TurnAndMove(s_Swerve, new Pose2d(new Translation2d(Units.metersToInches(-1), 0), new Rotation2d(0)), 0, 2.4),
           new TurnAndMove(s_Swerve, new Pose2d(new Translation2d(Units.metersToInches(1), 0), new Rotation2d(180)), 0, 2.4),
           new TurnAndMove(s_Swerve, new Pose2d(new Translation2d(Units.metersToInches(-1), 0), new Rotation2d(0)), 0, 2.4),
           new TurnAndMove(s_Swerve, new Pose2d(new Translation2d(Units.metersToInches(1), 0), new Rotation2d(180)), 0, 2.4),
           new TurnAndMove(s_Swerve, new Pose2d(new Translation2d(Units.metersToInches(-1), 0), new Rotation2d(0)), 0, 2.4),
           new TurnAndMove(s_Swerve, new Pose2d(new Translation2d(Units.metersToInches(1), 0), new Rotation2d(180)), 0, 2.4));
    }
}
