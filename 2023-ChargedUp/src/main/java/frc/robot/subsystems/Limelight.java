package frc.robot.subsystems;

import java.security.Key;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase 
{    
    public final double errorNum = -1000000.0; // an impossible number
    /* old code. Leaving in here just in case it's needed again. Remove before final version. */
    //private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //private NetworkTableEntry tx = table.getEntry("tx");
    //private NetworkTableEntry ty = table.getEntry("ty");
    //private NetworkTableEntry ta = table.getEntry("ta");
    //private NetworkTableEntry pipeline = table.getEntry("pipeline");
    //private NetworkTableEntry led = table.getEntry("ledMode");

    //One method to switch profile, take in double
    public void switchProfile(String cam, Constants.LimelightConstants.LimelightPipelines profile)
    {
        NetworkTableInstance.getDefault().getTable(cam).getEntry("pipeline").setNumber(profile.ordinal());
    }

    //One method to toggle LED, take in no parameters
    public void toggleLED(String cam)
    {
        if (NetworkTableInstance.getDefault().getTable(cam).getEntry("ledMode").getInteger(0) == 1)
            NetworkTableInstance.getDefault().getTable(cam).getEntry("ledMode").setNumber(3);
        else
            NetworkTableInstance.getDefault().getTable(cam).getEntry("ledMode").setNumber(1);
    }

    //one method to get target x
    public double getX(String cam)
    {
        return NetworkTableInstance.getDefault().getTable(cam).getEntry("tx").getDouble(errorNum - 1);
    }

    public double getY(String cam)
    {
        return NetworkTableInstance.getDefault().getTable(cam).getEntry("ty").getDouble(errorNum - 1);
    }

    //one method to get target area 
    public double getDistance(String cam){
        // todo we will need a table of values to know how to convert a value to distance
        // this table can be made after testing our camera pipelines on the final frame
        // we may need a table for each goal hight
        return NetworkTableInstance.getDefault().getTable(cam).getEntry("ty").getDouble(errorNum - 1); 
    }

    //post to smart dashboard periodically
    @Override
    public void periodic()
    {
    }
}
