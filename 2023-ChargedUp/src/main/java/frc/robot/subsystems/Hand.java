package frc.robot.subsystems;

// No idea what imports go here but there needs to be some


// Subsystem initializes hardware and methods that are then going to be used in commands
public class Hand extends SubsystemBase
{
    
    Pneumatic pneumatic;  
    Motor motor; 

    public Grabber() 
    {
        pneumatic = new pneumatic(ID); // Initalize Pneumatic we need
        motor = new motor(CANID); // Initialize Motor we need

        // Is it always set to the same value bc we are only using that specific motor?
    }

    public void clamp() // Clamp down pneumatics
    {
        pneumatic.setState(true);
    }

    public void unClamp() // Unclamp pnemumatics
    {
        pneumatic.setState(false);
    }

    public void moveShooter(int time) // move shooter
    {
        motor.setSpeed(0.7, time);
    }




}
