package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.drive.RunTrajectoryCommand;
import frc.robot.commands.shoot.AlignmentCommand;
import frc.robot.commands.shoot.RotateToAngleCommand;
import frc.robot.commands.shoot.ZeroShootAngleCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import java.util.ArrayList;

public class PathManager {
    public static Command getFourBallAuto(DriveSubsystem driveSubsystem, ShootSubsystem shootSubsystem, CollectSubsystem collectSubsystem, XboxController shootController){
        AlignmentCommand autoAlignmentCommand = new AlignmentCommand(shootSubsystem, driveSubsystem, collectSubsystem, shootController);
        autoAlignmentCommand.setAutoModeEnableShoot(true);
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ZeroShootAngleCommand(shootSubsystem),
                                new RotateToAngleCommand(shootSubsystem, 75),
                                autoAlignmentCommand
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(collectSubsystem::enableIntake),
                                new RunTrajectoryCommand(driveSubsystem, TrajectoryGenerator.generateTrajectory(
                                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                        new ArrayList<>(),
                                        new Pose2d(2.15, 1.4, Rotation2d.fromDegrees(0)),
                                        // Pass config
                                        new TrajectoryConfig(4, 1)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveSubsystem.getKinematics())
                                )),
                                new InstantCommand(collectSubsystem::disableIntake),
                                new DelayCommand(0.4),
                                new InstantCommand(() -> shootSubsystem.setTransferMotorVelocity(1500)),
                                new DelayCommand(2),
                                new InstantCommand(()-> shootSubsystem.setTransferMotorVelocity(0)),
                                new InstantCommand(collectSubsystem::enableIntake),
                                new RunTrajectoryCommand(driveSubsystem, TrajectoryGenerator.generateTrajectory(
                                        new Pose2d(2.15, 1.4, Rotation2d.fromDegrees(0)),
                                        new ArrayList<>(),
                                        new Pose2d(5.35, 1.4, Rotation2d.fromDegrees(20)),
                                        // Pass config
                                        new TrajectoryConfig(



                                                4, 1)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveSubsystem.getKinematics())
                                )),
                                new DelayCommand(1.2),
                                new InstantCommand(collectSubsystem::disableIntake),
                                new CommandBase() {
                                    final Timer timer = new Timer();

                                    @Override
                                    public void initialize() {
                                        timer.reset();
                                        timer.start();

                                    }

                                    @Override
                                    public void execute() {
                                        driveSubsystem.driveCartesian(-0.5, 0, 0);
                                    }

                                    @Override
                                    public void end(boolean interrupted) {
                                        driveSubsystem.driveCartesian(0, 0, 0);
                                        timer.stop();
                                    }

                                    @Override
                                    public boolean isFinished() {
                                        return timer.hasElapsed(0.7);
                                    }
                                },
                                new DelayCommand(1),
                                new InstantCommand(() -> shootSubsystem.setTransferMotorVelocity(1500)),
                                new DelayCommand(5),
                                new InstantCommand(() -> shootSubsystem.setTransferMotorVelocity(0))
                        )
                )
        );
    }

    public static Command getTwoBallAuto(DriveSubsystem driveSubsystem, ShootSubsystem shootSubsystem, CollectSubsystem collectSubsystem, XboxController shootController){
        AlignmentCommand autoAlignmentCommand = new AlignmentCommand(shootSubsystem, driveSubsystem, collectSubsystem, shootController);
        autoAlignmentCommand.setAutoModeEnableShoot(true);
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ZeroShootAngleCommand(shootSubsystem),
                                new RotateToAngleCommand(shootSubsystem, 75),
                                autoAlignmentCommand
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(collectSubsystem::enableIntake),
                                new RunTrajectoryCommand(driveSubsystem, TrajectoryGenerator.generateTrajectory(
                                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                        new ArrayList<>(),
                                        new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
                                        // Pass config
                                        new TrajectoryConfig(4, 1)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveSubsystem.getKinematics())
                                )),
                                new InstantCommand(collectSubsystem::disableIntake),
                                new DelayCommand(0.4),
                                new InstantCommand(() -> shootSubsystem.setTransferMotorVelocity(1500)),
                                new DelayCommand(4),
                                new InstantCommand(()-> shootSubsystem.setTransferMotorVelocity(0))
                        )
                )
        );
    }
}
