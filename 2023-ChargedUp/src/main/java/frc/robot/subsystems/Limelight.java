package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase 
{    
    public final double errorNum = -1000000.0; // an impossible number
    

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
        return NetworkTableInstance.getDefault().getTable(cam).getEntry("ty").getDouble(errorNum - 1); 
    }

    @Override
    public void periodic()
    {
    }
}
