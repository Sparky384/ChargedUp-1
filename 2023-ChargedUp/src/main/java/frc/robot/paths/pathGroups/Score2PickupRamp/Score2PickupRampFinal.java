package frc.robot.paths.pathGroups.Score2PickupRamp;

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
import frc.robot.paths.Pickup1st;
import frc.robot.paths.Pickup2nd;
import frc.robot.paths.ReturnFromPickup1st;

// Right Side Of The Arena

public class Score2PickupRampFinal extends SequentialCommandGroup {
    private static Swerve s_Swerve;
        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve s) {
        s_Swerve = s;

        return new SequentialCommandGroup(
        Pickup1st.followTrajectoryCommand(true, s_Swerve),
        ReturnFromPickup1st.followTrajectoryCommand(false, s_Swerve),
        Pickup2nd.followTrajectoryCommand(false, s_Swerve),
        Score2PickupRamp_4.followTrajectoryCommand(false, s_Swerve)
        );
    }
}
