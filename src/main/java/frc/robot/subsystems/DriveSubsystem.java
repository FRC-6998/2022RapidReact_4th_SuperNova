package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax motorFrontLeft = new CANSparkMax(Constants.MOTOR_CHASSIS_FL, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax motorFrontRight = new CANSparkMax(Constants.MOTOR_CHASSIS_FR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax motorRearLeft = new CANSparkMax(Constants.MOTOR_CHASSIS_RL, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax motorRearRight = new CANSparkMax(Constants.MOTOR_CHASSIS_RR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.FF_CHASSIS_KS, Constants.FF_CHASSIS_KV, Constants.FF_CHASSIS_KA);

    // NavX
    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    // Mecanum Drive Kinematics
    private static final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
            new Translation2d(Constants.LENGTH / 2, Constants.WIDTH / 2),
            new Translation2d(Constants.LENGTH / 2, -Constants.WIDTH / 2),
            new Translation2d(-Constants.LENGTH / 2, Constants.WIDTH / 2),
            new Translation2d(-Constants.LENGTH / 2, -Constants.WIDTH / 2)
    );

    // Mecanum Drive Odometry
    private final MecanumDriveOdometry odometry;

    private double deadband = Constants.CHASSIS_DEADBAND;

    public DriveSubsystem() {
        resetYaw();

        // Odometry
        odometry = new MecanumDriveOdometry(kinematics, navX.getRotation2d());
        // Restore motor controllers to factory defaults
        motorFrontLeft.restoreFactoryDefaults();
        motorFrontRight.restoreFactoryDefaults();
        motorRearLeft.restoreFactoryDefaults();
        motorRearRight.restoreFactoryDefaults();
        // Set current limit
        motorFrontLeft.setSmartCurrentLimit(Constants.CURRENT_LIMIT_CHASSIS);
        motorFrontRight.setSmartCurrentLimit(Constants.CURRENT_LIMIT_CHASSIS);
        motorRearLeft.setSmartCurrentLimit(Constants.CURRENT_LIMIT_CHASSIS);
        motorRearRight.setSmartCurrentLimit(Constants.CURRENT_LIMIT_CHASSIS);
        // Set idle mode
        motorFrontLeft.setIdleMode(Constants.IDLE_MODE);
        motorFrontRight.setIdleMode(Constants.IDLE_MODE);
        motorRearLeft.setIdleMode(Constants.IDLE_MODE);
        motorRearRight.setIdleMode(Constants.IDLE_MODE);
        // Set invert
        motorFrontLeft.setInverted(Constants.MOTOR_CHASSIS_FL_INVERTED);
        motorFrontRight.setInverted(Constants.MOTOR_CHASSIS_FR_INVERTED);
        motorRearLeft.setInverted(Constants.MOTOR_CHASSIS_RL_INVERTED);
        motorRearRight.setInverted(Constants.MOTOR_CHASSIS_RR_INVERTED);

        // Convert encoder value's units from rpm to meter per second(1/gearing*distance per rotation/min to sec)
        double convertFactor = 1 / Constants.CHASSIS_GEARING * Constants.DISTANCE_METER_PER_ROTATION / 60;
        motorFrontLeft.getEncoder().setPositionConversionFactor(convertFactor);
        motorFrontRight.getEncoder().setPositionConversionFactor(convertFactor);
        motorRearLeft.getEncoder().setPositionConversionFactor(convertFactor);
        motorRearRight.getEncoder().setPositionConversionFactor(convertFactor);
        motorFrontLeft.getEncoder().setVelocityConversionFactor(convertFactor);
        motorFrontRight.getEncoder().setVelocityConversionFactor(convertFactor);
        motorRearLeft.getEncoder().setVelocityConversionFactor(convertFactor);
        motorRearRight.getEncoder().setVelocityConversionFactor(convertFactor);

        // Scale P from sys-id to spark max
        motorFrontLeft.getPIDController().setP(Constants.PID_CHASSIS[0] * convertFactor);
        motorFrontRight.getPIDController().setP(Constants.PID_CHASSIS[0] * convertFactor);
        motorRearLeft.getPIDController().setP(Constants.PID_CHASSIS[0] * convertFactor);
        motorRearRight.getPIDController().setP(Constants.PID_CHASSIS[0] * convertFactor);

        // Set max acceleration
        motorFrontLeft.getPIDController().setSmartMotionMaxAccel(Constants.MAX_CHASSIS_ACCELERATION_METER_PETER_SECOND_SQUARE, 0);
        motorFrontRight.getPIDController().setSmartMotionMaxAccel(Constants.MAX_CHASSIS_ACCELERATION_METER_PETER_SECOND_SQUARE, 0);
        motorRearLeft.getPIDController().setSmartMotionMaxAccel(Constants.MAX_CHASSIS_ACCELERATION_METER_PETER_SECOND_SQUARE, 0);
        motorRearRight.getPIDController().setSmartMotionMaxAccel(Constants.MAX_CHASSIS_ACCELERATION_METER_PETER_SECOND_SQUARE, 0);
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), getWheelSpeeds());
        if (Constants.DEBUG) {
            ChassisSpeeds speed = kinematics.toChassisSpeeds(getWheelSpeeds());
            SmartDashboard.putNumber("DriveSubsystem ChassisSpeeds Vx", speed.vxMetersPerSecond);
            SmartDashboard.putNumber("DriveSubsystem ChassisSpeeds Vy", speed.vyMetersPerSecond);
            SmartDashboard.putNumber("DriveSubsystem ChassisSpeeds Omega Degrees", Math.toDegrees(speed.omegaRadiansPerSecond));
            Pose2d pose = odometry.getPoseMeters();
            SmartDashboard.putNumber("DriveSubsystem Odometry Px", pose.getX());
            SmartDashboard.putNumber("DriveSubsystem Odometry Py", pose.getY());
            SmartDashboard.putNumber("DriveSubsystem Odometry Rotation Degrees", pose.getRotation().getDegrees());
        }
    }

    public void resetYaw() {
        navX.reset();
    }


    public AHRS getNavX() {
        return navX;
    }

    public double getAngle(){
        return navX.getAngle();
    }

    public double getFusedHeading(){
        return navX.getFusedHeading();
    }

    public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
        driveCartesian(ySpeed, xSpeed, zRotation, 0.0);
    }

    public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
        ySpeed = MathUtil.applyDeadband(ySpeed, deadband);
        xSpeed = MathUtil.applyDeadband(xSpeed, deadband);
        zRotation = MathUtil.applyDeadband(zRotation, deadband);

        ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

        // Compensate for gyro angle.
        @SuppressWarnings("SuspiciousNameCombination")
        Vector2d input = new Vector2d(ySpeed, xSpeed);
        input.rotate(-gyroAngle);

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[RobotDriveBase.MotorType.kFrontLeft.value] = input.x + input.y + zRotation;
        wheelSpeeds[RobotDriveBase.MotorType.kFrontRight.value] = input.x - input.y - zRotation;
        wheelSpeeds[RobotDriveBase.MotorType.kRearLeft.value] = input.x - input.y + zRotation;
        wheelSpeeds[RobotDriveBase.MotorType.kRearRight.value] = input.x + input.y - zRotation;

        normalize(wheelSpeeds);

        MecanumDrive.WheelSpeeds speeds = new MecanumDrive.WheelSpeeds(
                wheelSpeeds[RobotDriveBase.MotorType.kFrontLeft.value],
                wheelSpeeds[RobotDriveBase.MotorType.kFrontRight.value],
                wheelSpeeds[RobotDriveBase.MotorType.kRearLeft.value],
                wheelSpeeds[RobotDriveBase.MotorType.kRearRight.value]);

        speeds.frontLeft *= Constants.MAX_CHASSIS_VELOCITY_METER_PER_SECOND;
        speeds.frontRight *= Constants.MAX_CHASSIS_VELOCITY_METER_PER_SECOND;
        speeds.rearLeft *= Constants.MAX_CHASSIS_VELOCITY_METER_PER_SECOND;
        speeds.rearRight *= Constants.MAX_CHASSIS_VELOCITY_METER_PER_SECOND;

        driveSpeeds(new MecanumDriveWheelSpeeds(speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight));
    }

    public void driveSpeeds(MecanumDriveWheelSpeeds speeds) {
        motorFrontLeft.getPIDController().setReference(
                speeds.frontLeftMetersPerSecond,
                CANSparkMax.ControlType.kVelocity,
                0,
                feedForward.calculate(speeds.frontLeftMetersPerSecond)
        );
        motorFrontRight.getPIDController().setReference(
                speeds.frontRightMetersPerSecond,
                CANSparkMax.ControlType.kVelocity,
                0,
                feedForward.calculate(speeds.frontRightMetersPerSecond)
        );
        motorRearLeft.getPIDController().setReference(
                speeds.rearLeftMetersPerSecond,
                CANSparkMax.ControlType.kVelocity,
                0,
                feedForward.calculate(speeds.rearLeftMetersPerSecond)
        );
        motorRearRight.getPIDController().setReference(
                speeds.rearRightMetersPerSecond,
                CANSparkMax.ControlType.kVelocity,
                0,
                feedForward.calculate(speeds.rearRightMetersPerSecond)
        );
    }

    /**
     * Stop chassis motors.
     */
    public void stopMotors() {
        motorFrontLeft.stopMotor();
        motorFrontRight.stopMotor();
        motorRearLeft.stopMotor();
        motorRearRight.stopMotor();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                motorFrontLeft.getEncoder().getVelocity(),
                motorFrontRight.getEncoder().getVelocity(),
                motorRearLeft.getEncoder().getVelocity(),
                motorRearRight.getEncoder().getVelocity()
        );
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, navX.getRotation2d());
    }

    public void resetEncoders() {
        motorFrontLeft.getEncoder().setPosition(0);
        motorFrontRight.getEncoder().setPosition(0);
        motorRearLeft.getEncoder().setPosition(0);
        motorRearRight.getEncoder().setPosition(0);
    }

    public static MecanumDriveKinematics getKinematics() {
        return kinematics;
    }

    public double getTurnRate() {
        return navX.getRate();
    }

    public Rotation2d getRotation2d() {
        return navX.getRotation2d();
    }

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    protected static void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }
}

