package frc.robot.commands.collect;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectSubsystem;


public class IntakeCommand extends CommandBase {
    private final CollectSubsystem collectSubsystem;
    private final XboxController driveController;
    private final XboxController shootController;

    public IntakeCommand(CollectSubsystem collectSubsystem, XboxController driveController, XboxController shootController) {
        this.collectSubsystem = collectSubsystem;
        this.driveController = driveController;
        this.shootController = shootController;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.collectSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (!DriverStation.isTeleopEnabled()) return;
        if (driveController.getBButton() || shootController.getBButton()) {
            collectSubsystem.enableIntake(false);
        } else if (shootController.getXButton()) {
            collectSubsystem.enableIntake(true);
        } else if (driveController.getXButton()) {
            collectSubsystem.reverseIntake();
        } else {
            collectSubsystem.disableIntake();
        }
    }
}
