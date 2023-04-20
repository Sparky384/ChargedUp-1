package frc.robot.commands.SliderFunctionality;
import edu.wpi.first.wpilibj.Timer;
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
        System.out.println("MoveSlider Init");
        timer.stop();
        timer.reset();
        timer.start();
    }

    public void execute() {
        if (timer.hasElapsed(delay)) {
            sliderSubsystem.move(sliderDistance);
            System.out.println("MoveSlider Executing"); 
        }
    }

    public boolean isFinished(){
        if (Math.abs(sliderSubsystem.getDistance() - sliderDistance) < Constants.sliderThreshold){
            System.out.println("MoveSlider Finished");
            return true; 
        } else {
            return false;
        }
    }
    @Override
    public void end(boolean interruped){
        System.out.println("MoveSlider interrupted");
        sliderSubsystem.stop();
    }
}
