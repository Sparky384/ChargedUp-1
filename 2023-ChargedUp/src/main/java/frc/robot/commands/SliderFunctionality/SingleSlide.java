package frc.robot.commands.SliderFunctionality;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Slider;


public class SingleSlide extends CommandBase {

    private Slider sliderSubsystem; 
    private double sliderDistance;
    private Timer timer;
    private double delay;
    private boolean executed;

    public SingleSlide(Slider slider, double initDistance, double delay) {
        sliderDistance = initDistance;
        sliderSubsystem = slider;
        this.delay = delay;
        timer = new Timer();
        addRequirements(sliderSubsystem);
        System.out.println("Have constructed slider command");
    }

    public void initialize() {
        System.out.println("Move slider command init to dist " + sliderDistance);
        timer.stop();
        timer.reset();
        timer.start();
        executed = false;
    }

    public void execute() {
        if (timer.hasElapsed(delay) && executed == false){
            sliderSubsystem.move(sliderDistance);
            executed = true;
        }

    }

    public boolean isFinished(){
        if (Math.abs(sliderSubsystem.getDistance() - sliderDistance) < Constants.sliderThreshold){
            System.out.println("Finished slider command at distance " + sliderSubsystem.getDistance());
            return true; 
        } else {
            return false;
        }
    }
    @Override
    public void end(boolean interruped){
        System.out.println("Stopping slider command");
        sliderSubsystem.stop();
    }
}
