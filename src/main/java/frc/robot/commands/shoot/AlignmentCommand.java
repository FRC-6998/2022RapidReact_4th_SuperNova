package frc.robot.commands.shoot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShootSubsystem;


public class AlignmentCommand extends CommandBase {
    private final ShootSubsystem shootSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final CollectSubsystem collectSubsystem;
    private final XboxController shootController;
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    private boolean lastHasTarget = false;
    private boolean lastShootEnabled = false;
    private boolean autoModeEnableShoot = false;
    private boolean enableAutoAlignment = true;
    private double targetDistance = 0;
    private double extraRPM = 0;
    private double xOffset = 0;

    public AlignmentCommand(ShootSubsystem shootSubsystem, DriveSubsystem driveSubsystem, CollectSubsystem collectSubsystem, XboxController shootController) {
        this.shootSubsystem = shootSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.shootController = shootController;
        this.collectSubsystem = collectSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shootSubsystem);
    }

    @Override
    public void initialize() {
        shootSubsystem.stopAllMotors();
        lastHasTarget = false;
        enableAutoAlignment = true;
        targetDistance = 0;
    }

    @Override
    public void execute() {
        boolean connected = isLimeLightConnected();
        boolean hasTarget = isTargetExists();
        boolean shootEnabled = shootController.getRightBumper() || (DriverStation.isAutonomousEnabled() && autoModeEnableShoot);
        if (enableAutoAlignment && connected && hasTarget) {
            lastHasTarget = true;

            // get data from limelight
            double tx = limelight.getEntry("tx").getDouble(0) + xOffset; // diff angle on x-axis
            double ty = limelight.getEntry("ty").getDouble(0); // diff angle on y-axis

            // calculate distance in meters
            targetDistance = (Constants.AUTO_ALIGNMENT_GOAL_HEIGHT_METER - Constants.AUTO_ALIGNMENT_LENS_HEIGHT_METER) / Math.tan(Math.toRadians(Constants.AUTO_ALIGNMENT_MOUNT_ANGLE + ty));
            // the distance between surface and center of hub is 68 cm or 0.68 m length
            targetDistance += 0.68;

            double rotateSetpoint = shootSubsystem.getRotateMotorPosition() + tx;
            rotateSetpoint -= driveSubsystem.getTurnRate() * 18;

            // Predict next rotation
            // Absolute angle from field
            double shooterAbsAngle = shootSubsystem.getRotateAngle() + driveSubsystem.getAngle();
            while (shooterAbsAngle >= 360.0) shooterAbsAngle -= 360;
            while (shooterAbsAngle < 0) shooterAbsAngle += 360;
            SmartDashboard.putNumber("Angle", shooterAbsAngle);

//            ChassisSpeeds speeds = driveSubsystem.getChassisSpeeds();
//
//            if ((shooterAbsAngle>=90 && shooterAbsAngle<180) || (shooterAbsAngle>=270 &&shooterAbsAngle<360)){
//                speeds.vyMetersPerSecond*=-1;
//            }
//
//            lastDynamicFixAngle = speeds.vyMetersPerSecond*15;
//
//            SmartDashboard.putNumber("Dynamic Fix Angle", lastDynamicFixAngle);
//
//            rotateSetpoint -= lastDynamicFixAngle;

            shootSubsystem.setRotateMotorPosition(rotateSetpoint);


            //TODO y-axis

            if (shootEnabled) {
                double rpm = 31.381 * targetDistance * targetDistance + 160 * targetDistance + 2950 + extraRPM;
                shootSubsystem.setShootMotorsVelocity(rpm);
                if (DriverStation.isTeleopEnabled() && Math.abs(shootSubsystem.getShootMotorsVelocity() - rpm) <= 100) {
                    shootController.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
                } else {
                    shootController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                }
            } else {
                if (lastShootEnabled) {
                    shootSubsystem.stopShootMotors();
                    shootController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                }
            }
        } else {
            if (lastHasTarget) {
                lastHasTarget = false;
                targetDistance = 0;
                //shootSubsystem.stopRotateMotor();
                shootSubsystem.stopAngleMotor();
            }
            if (shootEnabled) {
                double rpm = 4000 + extraRPM;
                shootSubsystem.setShootMotorsVelocity(rpm);
                if (DriverStation.isTeleopEnabled() && Math.abs(shootSubsystem.getShootMotorsVelocity() - rpm) <= 100) {
                    shootController.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
                } else {
                    shootController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                }
            } else {
                if (lastShootEnabled) {
                    shootSubsystem.stopShootMotors();
                    shootController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                }
            }
        }
        if (DriverStation.isTeleopEnabled()) {
            if (shootController.getLeftBumper()) {
                shootSubsystem.setTransferMotorVelocity(1500);
            } else if (shootSubsystem.getBallDetector() && collectSubsystem.isIntakeEnabled()) {
                shootSubsystem.setTransferMotorVelocity(1500);
            } else {
                shootSubsystem.setTransferMotorVelocity(0);
            }
        }
        lastShootEnabled = shootEnabled;
        SmartDashboard.putNumber("Target Distance", targetDistance);
    }

    @Override
    public void end(boolean interrupted) {
        shootSubsystem.stopAllMotors();
    }

    public boolean isLimeLightConnected() {
        return limelight.getInstance().isConnected();
    }

    public boolean isTargetExists() {
        return limelight.getEntry("tv").getDouble(0) >= 1.0;
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    public void setAutoAlignment(boolean enable) {
        enableAutoAlignment = enable;
        updateCommand();
    }

    public boolean getAutoAlignment() {
        return enableAutoAlignment;
    }

    public void setAutoModeEnableShoot(boolean enable) {
        autoModeEnableShoot = enable;
    }

    public void setExtraRPM(double speed) {
        extraRPM = speed;
        updateCommand();
    }

    public double getExtraRPM() {
        return extraRPM;
    }

    public void setXOffset(double xOffset) {
        this.xOffset = xOffset;
        updateCommand();
    }

    public double getXOffset() {
        return xOffset;
    }

    public void updateCommand() {
        shootSubsystem.setDefaultCommand(this);
    }
}
