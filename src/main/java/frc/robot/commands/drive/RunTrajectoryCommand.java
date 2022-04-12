package frc.robot.commands.drive;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


import java.util.List;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;


public class RunTrajectoryCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private final HolonomicDriveController follower;
    private final MecanumDriveKinematics kinematics;
    private final DriveSubsystem drive;
    private final Rotation2d targetRot;

    public RunTrajectoryCommand(DriveSubsystem driveTrainSubsystem, Trajectory trajectory) {
        this.trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
        List<Trajectory.State> states = trajectory.getStates();
        this.targetRot = states.get(states.size()-1).poseMeters.getRotation();
        follower = new HolonomicDriveController(
                new PIDController(1,0,0),
                new PIDController(1,0,0),
                new ProfiledPIDController(1,0,0, new TrapezoidProfile.Constraints(6.28, 3.14)));
        kinematics = DriveSubsystem.getKinematics();
        drive = driveTrainSubsystem;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetOdometry(trajectory.getInitialPose());
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double curTime = timer.get();

        var targetWheelSpeeds =
                kinematics.toWheelSpeeds(
                        follower.calculate(drive.getPose(), trajectory.sample(curTime), targetRot));

        var frontLeftSpeedSetpoint = targetWheelSpeeds.frontLeftMetersPerSecond;
        var frontRightSpeedSetpoint = targetWheelSpeeds.frontRightMetersPerSecond;
        var rearLeftSpeedSetpoint = targetWheelSpeeds.rearLeftMetersPerSecond;
        var rearRightSpeedSetpoint = targetWheelSpeeds.rearRightMetersPerSecond;

        drive.driveSpeeds(new MecanumDriveWheelSpeeds(frontLeftSpeedSetpoint, frontRightSpeedSetpoint, rearLeftSpeedSetpoint, rearRightSpeedSetpoint));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.stopMotors();
    }
}
