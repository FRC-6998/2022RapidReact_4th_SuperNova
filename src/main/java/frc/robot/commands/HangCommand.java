package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangSubsystem;


public class HangCommand extends CommandBase {
    private final HangSubsystem hangSubsystem;
    private final double setpoint;

    public HangCommand(HangSubsystem hangSubsystem, double setpoint) {
        this.hangSubsystem = hangSubsystem;
        this.setpoint = setpoint;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.hangSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        hangSubsystem.setSmartMotionSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return Math.abs(hangSubsystem.getHangerPosition()-setpoint)<=1;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
