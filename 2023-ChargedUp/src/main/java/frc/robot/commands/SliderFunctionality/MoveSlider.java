package frc.robot.commands.SliderFunctionality;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Slider;


public class MoveSlider extends CommandBase {

    private Slider sliderSubsystem; 
    private double sliderDistance;

    public MoveSlider(Slider slider, double initDistance) {
        sliderDistance = initDistance;
        sliderSubsystem = slider;
        addRequirements(sliderSubsystem);

    }

    public void execute() {
        SmartDashboard.putNumber("sliderDistance", sliderSubsystem.getDistance());
        SmartDashboard.putBoolean("sliderRunner", true);
        sliderSubsystem.move(sliderDistance); 
    }

    public boolean isFinished(){
        if (Math.abs(sliderSubsystem.getDistance() - sliderDistance) < Constants.doubleThreshold){
            return true; 
        } else {
            return false;
        }
    }
    @Override
    public void end(boolean interruped){
        sliderSubsystem.stop();
        SmartDashboard.putBoolean("sliderRunner", false);
    }
}
