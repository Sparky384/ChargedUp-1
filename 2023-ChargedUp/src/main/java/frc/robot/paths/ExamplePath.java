package frc.robot.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class ExamplePath extends SequentialCommandGroup { //might extend CommandGroupBase

    public ExamplePath(Swerve s_Swerve){

        PathPlannerTrajectory exampleTrajectory = PathPlanner.loadPath("Example Path", new PathConstraints(1, 1)); //not sure what it thinks the file name is or how it gets the directory.
        boolean isFirstPath = true; //if I'm not mistaken you should just be able to pass in true for the first path and ignore it on any others.

        PPSwerveControllerCommand swerveControllerCommand =  new PPSwerveControllerCommand(
            exampleTrajectory, 
            s_Swerve::getPose, // Pose supplier
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(Constants.AutoConstants.kPThetaController, Constants.AutoConstants.kIThetaController, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            s_Swerve::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            s_Swerve // Requires this drive subsystem
        );
        
        addCommands(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
                if(isFirstPath){
                    s_Swerve.resetOdometry(exampleTrajectory.getInitialHolonomicPose());
                }
            }),
            swerveControllerCommand
        );
    }
}
