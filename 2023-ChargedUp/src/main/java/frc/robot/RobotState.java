package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }
    
    private SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field = new Field2d();

    private SwerveDriveOdometry driveOdometry;
    

    public void initializePoseEstimator(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, rotation, modulePositions, new Pose2d());
        field.setRobotPose(new Pose2d(1.9, 4.99, Rotation2d.fromDegrees(0)));
        SmartDashboard.putData(field);
        driveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, rotation, modulePositions);
    }

    public void recordDriveObservations(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(rotation, modulePositions);
        driveOdometry.update(rotation, modulePositions);
    }

    public void setFieldToVehicle(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d fieldToVehicle) {
        poseEstimator.resetPosition(rotation, modulePositions, fieldToVehicle);
        driveOdometry.resetPosition(rotation, modulePositions, fieldToVehicle);
    }

    public Pose2d getFieldToVehicle() {
        // SmartDashboard.putNumber("OdometryX", driveOdometry.getPoseMeters().getX());    
        // SmartDashboard.putNumber("OdometryY", driveOdometry.getPoseMeters().getY());
        // SmartDashboard.putNumber("OdometryTheta", driveOdometry.getPoseMeters().getRotation().getDegrees());
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        // field.setRobotPose(driveOdometry.getPoseMeters());
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getOdometryFieldToVehicle() {
        return driveOdometry.getPoseMeters();
    }
}
