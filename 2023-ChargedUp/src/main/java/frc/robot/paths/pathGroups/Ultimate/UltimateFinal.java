package frc.robot.paths.pathGroups.Ultimate;

import frc.lib.pathplanner.com.pathplanner.lib.PathPlanner;
import frc.lib.pathplanner.com.pathplanner.lib.PathPlannerTrajectory;
import frc.lib.pathplanner.com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Swerve;
import frc.lib.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.robot.Constants;


public class UltimateFinal extends SequentialCommandGroup{
    private static Swerve s_Swerve;
        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve s) {
        s_Swerve = s;

        return new SequentialCommandGroup(
        Ultimate_1.followTrajectoryCommand(true, s_Swerve),
        Ultimate_2.followTrajectoryCommand(false, s_Swerve),
        Ultimate_3.followTrajectoryCommand(false, s_Swerve),
        Ultimate_4.followTrajectoryCommand(false, s_Swerve),
        Ultimate_5.followTrajectoryCommand(false, s_Swerve)
        );
    }
}
