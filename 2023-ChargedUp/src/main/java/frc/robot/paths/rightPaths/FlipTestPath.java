package frc.robot.paths.rightPaths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.lib.pathplanner.com.pathplanner.lib.PathPlanner;
import frc.lib.pathplanner.com.pathplanner.lib.PathPlannerTrajectory;
import frc.lib.pathplanner.com.pathplanner.lib.commands.PPSwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class FlipTestPath extends SequentialCommandGroup {
    private static Swerve s_Swerve;

    static PathPlannerTrajectory trajectory = PathPlanner.loadPath("fliptest", new PathConstraints(Constants.AutoConstants.kPathMaxVelocity, Constants.AutoConstants.kPathMaxAcceleration));

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(boolean isFirstPath, Swerve s) {
        s_Swerve = s;
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                s_Swerve.resetOdometry(trajectory.getInitialHolonomicPose());
            }
            }),
            new PPSwerveControllerCommand(
                trajectory, 
                s_Swerve::getPose, // Pose supplier
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDController(Constants.AutoConstants.kPPathXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(Constants.AutoConstants.kPPathYController, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(Constants.AutoConstants.kPPathThetaController, Constants.AutoConstants.kIPathThetaController, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                s_Swerve::setModuleStates, // Module states consumer
                true, // Path changes based on team color. We set it to false. Defaults to true.
                s_Swerve // Requires this drive subsystem
            )
        );
    }
}
