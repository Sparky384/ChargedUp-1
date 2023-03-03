package frc.robot.paths.pathGroups.leftGroups;
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
import frc.robot.paths.rightPaths.RPickup2nd;
import frc.robot.paths.leftPaths.LScore2nd;

public class LScore2 extends SequentialCommandGroup {
    private static Swerve s_Swerve;
        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve s) {
        s_Swerve = s;

        return new SequentialCommandGroup(
        LScore1.followTrajectoryCommand(s_Swerve), //if we decide to move after score 1 this cannot be called here.
        //Parallel Command Group: move, sometime later bring elevator to height
        LScore2nd.followTrajectoryCommand(false, s_Swerve)
        //deploy slider
        //drop piece
        //retract slider
        );
    }
}
