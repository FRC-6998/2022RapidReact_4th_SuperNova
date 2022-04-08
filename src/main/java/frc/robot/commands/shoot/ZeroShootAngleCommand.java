package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;


public class ZeroShootAngleCommand extends CommandBase {
    private final ShootSubsystem shootSubsystem;
    private final Timer timer = new Timer();

    public ZeroShootAngleCommand(ShootSubsystem shootSubsystem) {
        this.shootSubsystem = shootSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shootSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        shootSubsystem.setAngleMotorOutput(-0.1);
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(Constants.ZERO_ANGLE_MAX_SECONDS)) return true;
        return shootSubsystem.getAngleReverseLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}
