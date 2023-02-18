package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class auto1 extends SequentialCommandGroup{
    
    public auto1(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters. MUST HAVE START POINT OF 0, 0, 0
        Trajectory Trajectory = 
        TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                //new Pose2d(Units.inchesToMeters(20), Units.inchesToMeters(0), new Rotation2d(0)),
                //new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(0), new Rotation2d(0)),
                //new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(0), new Rotation2d(0)),
                //new Pose2d(Units.inchesToMeters(80), Units.inchesToMeters(0), new Rotation2d(0)),
                //new Pose2d(Units.inchesToMeters(100), Units.inchesToMeters(0), new Rotation2d(0)),
                //new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(0), new Rotation2d(0)),
                //new Pose2d(Units.inchesToMeters(140), Units.inchesToMeters(0), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(140), Units.inchesToMeters(0), new Rotation2d(0))
            ), config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, Constants.AutoConstants.kIThetaController, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                Trajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        
        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(Trajectory.getInitialPose())),
            swerveControllerCommand            
        );

    }
}
