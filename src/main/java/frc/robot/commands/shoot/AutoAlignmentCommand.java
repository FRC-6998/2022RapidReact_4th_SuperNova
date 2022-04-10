package frc.robot.commands.shoot;

import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShootSubsystem;


public class AutoAlignmentCommand extends CommandBase {
    private final ShootSubsystem shootSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    private boolean lastHasTarget = false;
    private boolean enableAutoAlignment = true;
    private double targetDistance = 0;

    public AutoAlignmentCommand(ShootSubsystem shootSubsystem, DriveSubsystem driveSubsystem, XboxController shootController) {
        this.shootSubsystem = shootSubsystem;
        this.driveSubsystem = driveSubsystem;
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
        if (enableAutoAlignment && connected && hasTarget){
            lastHasTarget = true;

            // get data from limelight
            double tx = limelight.getEntry("tx").getDouble(0); // diff angle on x-axis
            double ty = limelight.getEntry("ty").getDouble(0); // diff angle on y-axis

            // calculate distance in meters
            targetDistance = (Constants.AUTO_ALIGNMENT_GOAL_HEIGHT_METER - Constants.AUTO_ALIGNMENT_LENS_HEIGHT_METER) / Math.tan(Math.toRadians(Constants.AUTO_ALIGNMENT_MOUNT_ANGLE + ty));
            // the distance between surface and center of hub is 68 cm or 0.68 m length
            targetDistance += 0.68;

            double rotateSetpoint = shootSubsystem.getRotateMotorPosition() + tx/360.0;
            rotateSetpoint -= driveSubsystem.getTurnRate()*0.05;
            shootSubsystem.setRotateMotorPosition(rotateSetpoint);

            //TODO y-axis alignment
        }else{
            if (lastHasTarget){
                lastHasTarget = false;
                targetDistance = 0;
                //shootSubsystem.stopRotateMotor();
                shootSubsystem.stopAngleMotor();
            }
        }
        SmartDashboard.putNumber("Target Distance", targetDistance);
    }

    @Override
    public void end(boolean interrupted) {
        shootSubsystem.stopAllMotors();
    }

    public boolean isLimeLightConnected(){
        return limelight.getInstance().isConnected();
    }

    public boolean isTargetExists(){
        return limelight.getEntry("tv").getDouble(0) >= 1.0;
    }

    public double getTargetDistance(){
        return targetDistance;
    }

    public void setAutoAlignment(boolean enable){
        enableAutoAlignment = enable;
    }
}
