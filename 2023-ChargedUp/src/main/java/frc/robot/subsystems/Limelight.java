package frc.robot.subsystems;

import java.security.Key;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");

    //One method to switch profile, take in double
    public void switchProfile(double profile){
        table.getEntry("pipeline").setNumber(profile);
    }

    //One method to toggle LED, take in no parameters
    public void toggleLED(){
        if (table.getEntry("ledMode").equals(1)) {
            table.getEntry("ledMode").setNumber(3);
        }
        else{
            table.getEntry("ledMode").setNumber(1);
        }
    }

    //one method to get target x
    public double getX(){
        return tx.getDouble(0.0);
    }
    //one method to get target area 
    public double getDistance(){
        //do some math on tx
        return ((ty.getDouble(0.0) - 14.1)/-0.559); 
    }
    public double getY(){
        //do some math on ty
        return ty.getDouble(0.0);
    }
    //post to smart dashboard periodically
    @Override
    public void periodic(){
        SmartDashboard.putNumber("LimelightX", getDistance());
        SmartDashboard.putNumber("limelightY", getY());
        SmartDashboard.putNumber("LimelightDistance", getDistance());
    }
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);
}
