package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;

import frc.robot.paths.rightPaths.Blue2ScoreRightScore;

public class DistrictTestAuto extends SequentialCommandGroup{
    private static Swerve s_Swerve;

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve swerve, Elevator elevator, Slider slider, Wrist wrist, Hand hand) {
        s_Swerve = swerve;

        return new SequentialCommandGroup(
        Blue2ScoreRightScore.followTrajectoryCommand(false, s_Swerve)
        );
    }
}
