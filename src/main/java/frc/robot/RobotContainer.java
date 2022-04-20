// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.collect.IntakeCommand;
import frc.robot.commands.drive.MecanumDriveCommand;
import frc.robot.commands.shoot.AlignmentCommand;
import frc.robot.commands.shoot.ZeroShootAngleCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import java.util.Objects;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Game Controllers
    private final XboxController driveController = new XboxController(0);
    private final XboxController shootController = new XboxController(1);
    private final XboxController controller3 = new XboxController(2);

    // Subsystems
    private final CollectSubsystem collectSubsystem = new CollectSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem();
    private final ShootSubsystem shootSubsystem = new ShootSubsystem();
    private final HangSubsystem hangSubsystem = new HangSubsystem();

    // Auto Path Chooser
    private final SendableChooser<String> pathChooser = new SendableChooser<>();

    private final AlignmentCommand alignmentCommand = new AlignmentCommand(shootSubsystem, drive, collectSubsystem, shootController);

    private final HangManager hangManager = new HangManager(hangSubsystem, shootSubsystem);

    private Command currentHangCommand = new SequentialCommandGroup();

    private final AddressableLED hangLightStrip = new AddressableLED(Constants.LED_PWM_PORT);
    private final AddressableLEDBuffer hangLightStripBuffer = new AddressableLEDBuffer(Constants.LED_LENGTH);

    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public static final String AUTO_FOUR_BALL = "4 Ball Auto";
    public static final String AUTO_TWO_BALL = "2 Ball Auto";

    private double extraShootSpeed = 0;
    private double rotateOffset = 0;

    private boolean hangFinished = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        hangLightStrip.setLength(hangLightStripBuffer.getLength());
        hangLightStrip.start();

        // Add path options and send to the dashboard
        pathChooser.setDefaultOption("4 Ball Auto", AUTO_FOUR_BALL);
        pathChooser.addOption("2 Ball Auto", AUTO_TWO_BALL);
        SmartDashboard.putData("Auto choices", pathChooser);

        drive.setDefaultCommand(new MecanumDriveCommand(drive, driveController));
        collectSubsystem.setDefaultCommand(new IntakeCommand(collectSubsystem, driveController, shootController));
        shootSubsystem.setDefaultCommand(alignmentCommand);

        CommandScheduler.getInstance().onCommandFinish(command -> {
            if (Objects.equals(command.getName(), "Hang")){
                hangFinished = true;
            }
        });
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

        // use window or menu button on drive controller to reset chassis heading
        new JoystickButton(driveController, 7).whenPressed(new InstantCommand(drive::resetYaw));
//        new JoystickButton(driveController, 8).whenPressed(new InstantCommand(drive::resetYaw));


        // use pov on shoot controller to adjust speed and rotation offset
        new POVButton(shootController, 0)
                .whenPressed(new InstantCommand(() -> alignmentCommand.setExtraRPM(alignmentCommand.getExtraRPM()+50)));
        new POVButton(shootController, 90)
                .whenPressed(new InstantCommand(() -> alignmentCommand.setXOffset(alignmentCommand.getXOffset()+1)));
        new POVButton(shootController, 180)
                .whenPressed(new InstantCommand(() -> alignmentCommand.setExtraRPM(alignmentCommand.getExtraRPM()-50)));
        new POVButton(shootController, 270)
                .whenPressed(new InstantCommand(() -> alignmentCommand.setXOffset(alignmentCommand.getXOffset()-1)));

        new JoystickButton(shootController, 8).whenPressed(new InstantCommand(()-> alignmentCommand.setAutoAlignment(!alignmentCommand.getAutoAlignment())));

        new JoystickButton(driveController, 5).whenPressed(new InstantCommand(() -> {
            currentHangCommand.cancel();
            hangFinished = false;
            currentHangCommand = hangManager.prevCommand().withName("Hang");
            currentHangCommand.schedule();
        }));

        new JoystickButton(driveController,8).whenPressed(new InstantCommand(() -> hangSubsystem.setHangerEnable(!hangSubsystem.getHangerEnable())));
        new JoystickButton(driveController, 6).whenPressed(new InstantCommand(() -> {
            if (hangSubsystem.getHangerEnable()){
                if (hangManager.getStep() == -1 || hangFinished){
                    hangFinished = false;
                    currentHangCommand = hangManager.nextCommand().withName("Hang");
                    currentHangCommand.schedule();
                }
            }
        }));

        new JoystickButton(controller3, 2).whenPressed(()->hangSubsystem.setIdleMode(CANSparkMax.IdleMode.kBrake));
        new JoystickButton(controller3, 3).whenPressed(()->hangSubsystem.setIdleMode(CANSparkMax.IdleMode.kCoast));

//        new JoystickButton(controller3, 6).whenPressed(
//                new SequentialCommandGroup(
//                        new RotateToAngleCommand(shootSubsystem, 270),
//                        new InstantCommand(hangSubsystem::extendSolenoid),
//                        new DelayCommand(0.5),
//                        new MoveToHangSetpointCommand(hangSubsystem, 60),
//                        new InstantCommand(hangSubsystem::stopMotors),
//                        new InstantCommand(hangSubsystem::resetEncoder)
//                )
//        );
 //       new JoystickButton(controller3, 5).whenPressed(new InstantCommand(compressor::disable).andThen(new MoveToHangSetpointCommand(hangSubsystem, 110)).andThen(new MoveToHangSetpointCommand(hangSubsystem, 65)).andThen(new DelayCommand(0.5)).andThen(new MoveToHangSetpointCommand(hangSubsystem, 205)).andThen(new MoveToHangSetpointCommand(hangSubsystem, 165)).andThen(new DelayCommand(0)).andThen(new MoveToHangSetpointCommand(hangSubsystem, 200)));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        switch (pathChooser.getSelected()){
            case AUTO_FOUR_BALL:
                return PathManager.getFourBallAuto(drive, shootSubsystem, collectSubsystem, shootController);
            case AUTO_TWO_BALL:
                return PathManager.getTwoBallAuto(drive, shootSubsystem, collectSubsystem, shootController);
            default:
                break;
        }
        return new InstantCommand();
    }

    public void robotInit() {
        drive.resetYaw();
        compressor.enableDigital();
    }

    public void autonomousInit() {
        enableLimeLightLED();
        hangSubsystem.stopSolenoid();
    }

    public void teleopInit() {
        hangManager.setStep(-1);
        enableLimeLightLED();
        new ZeroShootAngleCommand(shootSubsystem).schedule();
        extraShootSpeed = 0;
        hangFinished = false;
        hangSubsystem.startTimer();
        hangSubsystem.setHangerEnable(false);
    }

    public void testInit() {
    }

    public void robotPeriodic() {
        handleColor();
    }

    public void disableInit() {
        disableLimeLightLED();
        shootController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        hangSubsystem.stopMotors();
        hangSubsystem.stopSolenoid();
        hangSubsystem.resetTimer();
        hangSubsystem.setHangerEnable(false);
    }

    public void teleopPeriodic() {
        SmartDashboard.putNumber("Shoot Speed Offset", extraShootSpeed);
        SmartDashboard.putBoolean("Command Finish", hangFinished);
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

    public void enableLimeLightLED(){
        writeLimeLightLEDMode(false);
        writeLimeLightLEDMode(true);
    }

    public void disableLimeLightLED(){
        writeLimeLightLEDMode(true);
        writeLimeLightLEDMode(false);
    }


    private void writeLimeLightLEDMode(boolean enable){
        nt.getTable("limelight").getEntry("ledMode").setNumber(enable?3:1);
    }

}
