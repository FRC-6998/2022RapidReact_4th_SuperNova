package frc.robot.commands.shoot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;


public class RotateToAngleCommand extends CommandBase {
    private final ShootSubsystem shootSubsystem;
    private final Timer timer = new Timer();
    private final double angle;

    public RotateToAngleCommand(ShootSubsystem shootSubsystem, double angle) {
        this.shootSubsystem = shootSubsystem;
        this.angle = MathUtil.clamp(angle, 0, Constants.SHOOT_MAX_ROTATE_ANGLE);
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shootSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        shootSubsystem.setRotateMotorPosition(angle);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3)||Math.abs(shootSubsystem.getRotateMotorPosition()-angle)<=2;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (interrupted){
            shootSubsystem.stopRotateMotor();
        }
    }
}
