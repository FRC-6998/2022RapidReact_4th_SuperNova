package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;


public class MecanumDriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final XboxController controller;

    public MecanumDriveCommand(DriveSubsystem driveSubsystem, XboxController controller) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double ySpeed = -controller.getLeftY();
        double xSpeed = controller.getLeftX();
        double zRotation = controller.getRightX();
        double pov = controller.getPOV();
        ySpeed += 0.15 * getPovY(pov);
        xSpeed += 0.15 * getPovX(pov);
        double gyroAngle = Constants.ENABLE_FOD ? driveSubsystem.getAngle() : 0.0;
        driveSubsystem.driveCartesian(ySpeed, xSpeed, zRotation, gyroAngle);
    }

    public double getPovY(double pov){
        if (pov==315 || pov==0 || pov==45){
            return 1.0;
        }
        if (pov==135 || pov==180 || pov==225){
            return -1.0;
        }
        return 0.0;
    }
    public double getPovX(double pov){
        if (pov==45 || pov==90 || pov==135){
            return 1.0;
        }
        if (pov==225 || pov==270 || pov==315){
            return -1.0;
        }
        return 0.0;
    }
}
