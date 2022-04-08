package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DelayCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final double seconds;

    public DelayCommand(double seconds) {
        addRequirements();
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}
