package frc.robot.commands.SliderFunctionality;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Slider;


public class MoveSlider extends CommandBase {

    private Slider sliderSubsystem; 
    private double sliderDistance;
    private Timer timer;
    private double delay;

    public MoveSlider(Slider slider, double initDistance, double delay) {
        sliderDistance = initDistance;
        sliderSubsystem = slider;
        this.delay = delay;
        timer = new Timer();
        addRequirements(sliderSubsystem);
    }

    public void initialize() {
        timer.stop();
        timer.reset();
        timer.start();
    }

    public void execute() {
        if (timer.hasElapsed(delay))
            sliderSubsystem.move(sliderDistance); 
    }

    public boolean isFinished(){
        if (Math.abs(sliderSubsystem.getDistance() - sliderDistance) < Constants.sliderThreshold){
            return true; 
        } else {
            return false;
        }
    }
    @Override
    public void end(boolean interruped){
        sliderSubsystem.stop();
    }
}
