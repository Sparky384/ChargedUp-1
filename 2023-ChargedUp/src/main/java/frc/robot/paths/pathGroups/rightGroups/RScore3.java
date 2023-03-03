package frc.robot.paths.pathGroups.rightGroups;

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
import frc.robot.paths.rightPaths.RPickup3rd;
import frc.robot.paths.rightPaths.RScore3rd;

// Right Side Of The Arena

public class RScore3 extends SequentialCommandGroup {
    private static Swerve s_Swerve;
        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve s) {
        s_Swerve = s;

        return new SequentialCommandGroup(
        RScore2.followTrajectoryCommand(s_Swerve),
        //parallel command group: move, set elevator down sometime during the middle of this.
        RPickup3rd.followTrajectoryCommand(false, s_Swerve),
        //parallel command group: move, set elevator up to height.
        RScore3rd.followTrajectoryCommand(false, s_Swerve)
        //deploy slider
        //drop piece
        //retract slider
        //maybe move out after?
        );
    }
}
