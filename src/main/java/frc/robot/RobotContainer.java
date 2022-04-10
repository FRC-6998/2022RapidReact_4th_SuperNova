// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.MecanumDriveKinematicsConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.HangCommand;
import frc.robot.commands.collect.IntakeCommand;
import frc.robot.commands.drive.MecanumDriveCommand;
import frc.robot.commands.shoot.AutoAlignmentCommand;
import frc.robot.commands.shoot.ZeroShootAngleCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import java.util.ArrayList;
import java.util.List;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Game Controllers
    private final XboxController controller1 = new XboxController(0);
    private final XboxController controller2 = new XboxController(1);
    private final XboxController controller3 = new XboxController(2);

    // Subsystems
    private final CollectSubsystem intake = new CollectSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem();
    private final ShootSubsystem shoot = new ShootSubsystem();
    private final HangSubsystem hangSubsystem = new HangSubsystem();

    private final SendableChooser<String> pathChooser = new SendableChooser<>();

    private final AddressableLED hangLightStrip = new AddressableLED(9);
    private final AddressableLEDBuffer hangLightStripBuffer = new AddressableLEDBuffer(148);

    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public static final String AUTO_1 = "Auto 1";
    public static final String AUTO_2 = "Auto 2";
    public static final String AUTO_3 = "Auto 3";

    private boolean teleop = false;

    private double baseSpeed = 3700;
    private double xOffset = 0;

    private double testHang = 0;


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        hangLightStrip.setLength(hangLightStripBuffer.getLength());
        hangLightStrip.start();

        // Add path options and send to the dashboard
        pathChooser.setDefaultOption("Auto 1", AUTO_1);
        pathChooser.addOption("Auto 2", AUTO_2);
        pathChooser.addOption("Auto 3", AUTO_3);
        SmartDashboard.putData("Auto choices", pathChooser);

        drive.setDefaultCommand(new MecanumDriveCommand(drive, controller1));
        intake.setDefaultCommand(new IntakeCommand(intake, controller1, controller2));
        shoot.setDefaultCommand(new AutoAlignmentCommand(shoot, drive, controller2));
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
        new JoystickButton(controller1, 7).whenPressed(new InstantCommand(drive::resetYaw));

        new POVButton(controller2, 0)
                .whenPressed(new InstantCommand(() -> baseSpeed += 50));
        new POVButton(controller2, 90)
                .whenPressed(new InstantCommand(() -> xOffset -= 0.2));
        new POVButton(controller2, 180)
                .whenPressed(new InstantCommand(() -> baseSpeed -= 50));
        new POVButton(controller2, 270)
                .whenPressed(new InstantCommand(() -> xOffset += 0.2));

        new JoystickButton(controller1, XboxController.Button.kA.value).whenPressed(new InstantCommand(() -> {
            if (hangSubsystem.getSolenoid()){
                hangSubsystem.collapseSolenoid();
            }else{
                hangSubsystem.extendSolenoid();
            }
        }));

        new JoystickButton(controller2, XboxController.Button.kY.value).whenPressed(new InstantCommand(() -> baseSpeed+=50));

        new JoystickButton(controller2, XboxController.Button.kA.value).whenPressed(new InstantCommand(() -> baseSpeed-=50));

        //new JoystickButton(controller2, 8).whenPressed(new InstantCommand(() -> shoot.forceDisableAlignment(!shoot.isForceDisableAlignment())));

        // control shoot motors and transfer motor
//        shoot.setDefaultCommand(new RunCommand(() -> {
//            if (!teleop) return;
//            if (controller2.getLeftTriggerAxis() >= 0.2 || controller2.getRightTriggerAxis() >= 0.2) {
//                hang.disableCompressor();
//                shoot.enableShootMotor();
//            } else {
//                shoot.disableShootMotor();
//                hang.enableCompressor();
//            }
//            if (controller2.getRightTriggerAxis() >= 0.2) {
//                shoot.setTransferMotorSpeed(0.8);
//            } else if (controller2.getYButton()) {
//                shoot.setTransferMotorSpeed(-0.8);
//            } else {
//                shoot.stopTransferMotor();
//            }
//            double rot = controller2.getRightX();
//            rot = Math.abs(rot) >= 0.2 ? rot : 0;
//            if (rot != 0) {
//                shoot.overrideRotate(Math.copySign(0.5, rot));
//            } else {
//                shoot.cancelOverrideRotate();
//            }
//            shoot.setBaseSpeed(baseSpeed);
//            shoot.setXOffset(xOffset);
//            SmartDashboard.putNumber("Base Speed", baseSpeed);
//        }, shoot));


        new JoystickButton(controller3, 2).whenPressed(hangSubsystem::resetEncoder);

        new JoystickButton(controller3, 5).whenPressed(new InstantCommand(compressor::disable).andThen(new HangCommand(hangSubsystem,110)).andThen(new HangCommand(hangSubsystem, 65)).andThen(new DelayCommand(0.5)).andThen(new HangCommand(hangSubsystem, 210)).andThen(new HangCommand(hangSubsystem, 165)).andThen(new DelayCommand(0)).andThen(new HangCommand(hangSubsystem, 200)));
        new JoystickButton(controller3, XboxController.Button.kA.value).whenPressed(new InstantCommand(() -> hangSubsystem.setSmartMotionSetpoint(testHang-=10)));
        new JoystickButton(controller3, XboxController.Button.kY.value).whenPressed(new InstantCommand(() -> hangSubsystem.setSmartMotionSetpoint(testHang+=10)));

//        new JoystickButton(controller3, 3).whenPressed(new InstantCommand(() -> hangSubsystem.setSmartMotionSetpoint(110)));
//        new JoystickButton(controller3, 8).whenPressed(new InstantCommand(() -> hangSubsystem.setSmartMotionSetpoint(170)));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

    public void robotInit() {
        drive.resetYaw();
        compressor.enableDigital();
    }

    public void autonomousInit() {
        teleop = false;
    }

    public void teleopInit() {
        teleop = true;

        new ZeroShootAngleCommand(shoot).schedule();
//        shoot.setAlliance(alliance);
//        shoot.enableLimelightGreenLED();
//        shoot.forceDisableAlignment(false);

        hangSubsystem.resetEncoder();
        hangSubsystem.set(0);
        testHang = 0;
        shoot.stopAllMotors();
    }

    public void testInit() {
        teleop = false;
//        shoot.setAlliance(alliance);
//        shoot.enableLimelightGreenLED();

    }

    public void robotPeriodic() {
        handleColor();
    }

    public void disableInit() {
        teleop = false;
//        shoot.disableLimeLightGreenLED();
    }

    public void teleopPeriodic(){
        SmartDashboard.putNumber("Velocity", baseSpeed);
        shoot.setShootMotorsVelocity(controller2.getRightBumper()?baseSpeed:0);
        if (controller2.getLeftBumper()){
            shoot.setTrigger(1);
        }else{
            shoot.setTrigger(0);
        }

        //shoot.setAngleMotorOutput(0.3*(controller2.getRightTriggerAxis()- controller2.getLeftTriggerAxis()));
    }

    private void handleColor() {
//        if (teleop){
//            boolean hasTarget = shoot.hasTarget();
//            double distance = shoot.getTargetDistance();
//            double currentRPM = shoot.getShootMotorVelocity();
//            double targetRPM = shoot.getTargetVelocity();
//            boolean ready = hasTarget && currentRPM >= 60 && Math.abs(currentRPM - targetRPM) <= 45;
//            if (shoot.isForceDisableAlignment() || (distance >= 2.58 && distance <= 3.08)) {
//                // green
//                if (ready){
//                    // green blink
//                    blinkCount++;
//                    if (blinkCount>=6){
//                        blinkCount=0;
//                        lastBrightness = 100 - lastBrightness;
//                        for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
//                            hangLightStripBuffer.setRGB(i, 0, lastBrightness, 0);
//                        }
//                    }
//                }else{
//                    for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
//                        hangLightStripBuffer.setRGB(i, 0, 100, 0);
//                    }
//                    blinkCount=0;
//                    lastBrightness = 0;
//                }
//            } else if (distance >= 2.4 && distance <= 3.6) {
//                // yellow
//                if (ready){
//                    // yellow blink
//                    blinkCount++;
//                    if (blinkCount>=6){
//                        blinkCount=0;
//                        lastBrightness = 50 - lastBrightness;
//                        for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
//                            hangLightStripBuffer.setRGB(i, lastBrightness, lastBrightness, 0);
//                        }
//                    }
//                }else{
//                    for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
//                        hangLightStripBuffer.setRGB(i, 50, 50, 0);
//                    }
//                    blinkCount = 0;
//                    lastBrightness = 0;
//                }
//            } else {
//                // alliance color
//                for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
//                    hangLightStripBuffer.setRGB(i, alliance == DriverStation.Alliance.Red ? 100 : 0, 0, alliance == DriverStation.Alliance.Blue ? 100 : 0);
//                }
//            }
//        }else{
//            for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
//                hangLightStripBuffer.setRGB(i, alliance == DriverStation.Alliance.Red ? 100 : 0, 0, alliance == DriverStation.Alliance.Blue ? 100 : 0);
//            }
//        }
//        hangLightStrip.setData(hangLightStripBuffer);
    }
}
