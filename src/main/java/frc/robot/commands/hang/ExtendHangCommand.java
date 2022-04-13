package frc.robot.commands.hang;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangSubsystem;


public class ExtendHangCommand extends CommandBase {
    private final HangSubsystem hangSubsystem;
    private final Timer timer = new Timer();

    public ExtendHangCommand(HangSubsystem hangSubsystem) {
        this.hangSubsystem = hangSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.hangSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        hangSubsystem.resetEncoder();
        hangSubsystem.extendSolenoid();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(0.5)){
            hangSubsystem.setSmartMotionSetpoint(60);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(hangSubsystem.getHangerPosition()-60)<=2 || timer.hasElapsed(5);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}
