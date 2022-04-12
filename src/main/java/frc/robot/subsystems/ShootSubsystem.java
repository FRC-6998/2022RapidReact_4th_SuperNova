package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShootSubsystem extends SubsystemBase {
    private final WPI_TalonFX shootMotorUp = new WPI_TalonFX(Constants.MOTOR_SHOOT_MAIN_UP);
    private final WPI_TalonFX shootMotorDown = new WPI_TalonFX(Constants.MOTOR_SHOOT_MAIN_DOWN);
    private final CANSparkMax triggerMotor = new CANSparkMax(Constants.MOTOR_SHOOT_TRIGGER, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax transferMotor = new CANSparkMax(Constants.MOTOR_TRANSFER, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax angleMotor = new CANSparkMax(Constants.MOTOR_SHOOT_ANGLE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rotateMotor = new CANSparkMax(Constants.MOTOR_SHOOT_ROTATE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SparkMaxLimitSwitch angleFwdLimit;
    private final SparkMaxLimitSwitch angleRevLimit;
    private final SparkMaxLimitSwitch ballDetector;

    private final Timer timer = new Timer();

    public ShootSubsystem() {
        // Reset motor controllers to factory defaults
        shootMotorUp.configFactoryDefault();
        shootMotorDown.configFactoryDefault();
        rotateMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();
        triggerMotor.restoreFactoryDefaults();
        transferMotor.restoreFactoryDefaults();

        // Configure invert on motors
        shootMotorUp.setInverted(Constants.MOTOR_SHOOT_UP_INVERTED);
        shootMotorDown.setInverted(Constants.MOTOR_SHOOT_DOWN_INVERTED);
        rotateMotor.setInverted(Constants.MOTOR_SHOOT_ROTATE_INVERTED);
        angleMotor.setInverted(Constants.MOTOR_SHOOT_ANGLE_INVERTED);
        triggerMotor.setInverted(Constants.MOTOR_SHOOT_TRIGGER_INVERTED);
        transferMotor.setInverted(Constants.MOTOR_TRANSFER_INVERTED);

        // Configure neutral/idle mode on motors
        shootMotorUp.setNeutralMode(NeutralMode.Coast);
        shootMotorDown.setNeutralMode(NeutralMode.Coast);
        rotateMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        triggerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        transferMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        // Configure current limit on motors
        shootMotorUp.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 39, 0, 0));
        shootMotorDown.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 39, 0, 0));
        rotateMotor.setSmartCurrentLimit(30);
        angleMotor.setSmartCurrentLimit(30);
        triggerMotor.setSmartCurrentLimit(30);
        transferMotor.setSmartCurrentLimit(30);

        // Configure PID controller on motors
        shootMotorUp.config_kP(0, Constants.PID_SHOOT_MAIN[0]);
        shootMotorUp.config_kI(0, Constants.PID_SHOOT_MAIN[1]);
        shootMotorUp.config_kD(0, Constants.PID_SHOOT_MAIN[2]);
        shootMotorUp.config_kF(0, Constants.FF_SHOOT_MAIN);
        shootMotorDown.config_kP(0, Constants.PID_SHOOT_MAIN[0]);
        shootMotorDown.config_kI(0, Constants.PID_SHOOT_MAIN[1]);
        shootMotorDown.config_kD(0, Constants.PID_SHOOT_MAIN[2]);
        shootMotorDown.config_kF(0, Constants.FF_SHOOT_MAIN);

        shootMotorUp.setSafetyEnabled(false);
        shootMotorDown.setSafetyEnabled(false);


        // set conversion factor by gearing
        double rotateFactor = 1 / Constants.SHOOT_ROTATE_GEARING * 360;
        rotateMotor.getEncoder().setPositionConversionFactor(rotateFactor);
        rotateMotor.getEncoder().setVelocityConversionFactor(rotateFactor);

        SparkMaxPIDController anglePIDController = angleMotor.getPIDController();
        anglePIDController.setP(Constants.PID_SHOOT_ANGLE[0]);
        anglePIDController.setI(Constants.PID_SHOOT_ANGLE[1]);
        anglePIDController.setD(Constants.PID_SHOOT_ANGLE[2]);
        anglePIDController.setFF(Constants.FF_SHOOT_ANGLE);
        anglePIDController.setSmartMotionMaxVelocity(Constants.SMART_MOTION_MAX_VELOCITY_SHOOT_ANGLE, 0);
        anglePIDController.setSmartMotionMaxAccel(Constants.SMART_MOTION_MAX_ACCEL_SHOOT_ANGLE, 0);

        SparkMaxPIDController rotatePIDController = rotateMotor.getPIDController();
        rotatePIDController.setP(Constants.PID_SHOOT_ROTATE[0] / 360);
        rotatePIDController.setI(Constants.PID_SHOOT_ROTATE[1] / 360);
        rotatePIDController.setD(Constants.PID_SHOOT_ROTATE[2] / 360);
        rotatePIDController.setFF(Constants.FF_SHOOT_ROTATE / 360);
        rotatePIDController.setSmartMotionMaxVelocity(Constants.SMART_MOTION_MAX_VELOCITY_SHOOT_ROTATE*360, 0);
        rotatePIDController.setSmartMotionMaxAccel(Constants.SMART_MOTION_MAX_ACCEL_SHOOT_ROTATE*360, 0);

        SparkMaxPIDController triggerPIDController = triggerMotor.getPIDController();
        triggerPIDController.setP(Constants.PID_SHOOT_TRIGGER[0]);
        triggerPIDController.setI(Constants.PID_SHOOT_TRIGGER[1]);
        triggerPIDController.setD(Constants.PID_SHOOT_TRIGGER[2]);
        triggerPIDController.setFF(Constants.FF_SHOOT_TRIGGER);

        SparkMaxPIDController transferPIDController = transferMotor.getPIDController();
        transferPIDController.setP(Constants.PID_SHOOT_TRANSFER[0]);
        transferPIDController.setI(Constants.PID_SHOOT_TRANSFER[1]);
        transferPIDController.setD(Constants.PID_SHOOT_TRANSFER[2]);
        transferPIDController.setFF(Constants.FF_SHOOT_TRANSFER);

        angleFwdLimit = angleMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        angleRevLimit = angleMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        // Enable hard limit switch
        angleFwdLimit.enableLimitSwitch(false);
        angleRevLimit.enableLimitSwitch(true);
        // Enable soft limit switch
        angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.SOFT_LIMIT_FWD_SHOOT_ANGLE);
        angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

//        rotateMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.SHOOT_MAX_ROTATE_ANGLE);
//        rotateMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
//        rotateMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
//        rotateMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        rotateMotor.set(0);
        rotateMotor.getEncoder().setPosition(0);

        angleMotor.getEncoder().setPosition(0);

        SparkMaxLimitSwitch triggerFwdLimit = triggerMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        triggerFwdLimit.enableLimitSwitch(false);
        ballDetector = triggerFwdLimit;
    }

    @Override
    public void periodic() {
        if (angleRevLimit.isPressed()) {
            angleMotor.getEncoder().setPosition(0);
        }
        SmartDashboard.putNumber("Current Velocity", getShootMotorsVelocity() * 600 / 2048);
    }

    public void setShootMotorsVelocity(double rpm) {
        double targetVelocity100ms = rpm * 2048.0 / 600.0;
        shootMotorUp.set(ControlMode.Velocity, targetVelocity100ms);
        shootMotorDown.set(ControlMode.Velocity, targetVelocity100ms);
        triggerMotor.getPIDController().setReference(rpm == 0 ? 0 : 2500, CANSparkMax.ControlType.kVelocity);
    }

    public double getShootMotorsVelocity() {
        return (shootMotorUp.getSelectedSensorVelocity() + shootMotorDown.getSelectedSensorVelocity()) / 2.0 * 600/2048;
    }

    public void stopShootMotors() {
        shootMotorUp.set(ControlMode.PercentOutput, 0);
        shootMotorDown.set(ControlMode.PercentOutput, 0);
        triggerMotor.set(0);
    }

    public void setRotateMotorPosition(double position) {
        if (position>=360){
            while (position >= 360) {
                position -= 360;
            }
        }else if(position>=Constants.SHOOT_MAX_ROTATE_ANGLE+10) {
            position = 0;
        }else{
            while (position < 0) {
                position += 360;
            }
        }
        position = MathUtil.clamp(position, 0, Constants.SHOOT_MAX_ROTATE_ANGLE);
        rotateMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kSmartMotion);
    }

    public double getRotateMotorPosition() {
        return rotateMotor.getEncoder().getPosition();
    }

    public double getRotateAngle(){
        return getRotateMotorPosition()-270.0;
    }

    public void stopRotateMotor() {
        rotateMotor.stopMotor();
    }

    public void setAngleMotorPosition(double position) {
        angleMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kSmartMotion);
    }

    public void setAngleMotorOutput(double output) {
        angleMotor.getPIDController().setReference(output, CANSparkMax.ControlType.kDutyCycle);
    }

    public double getAngleMotorPosition() {
        return angleMotor.getEncoder().getPosition();
    }

    public void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    public void stopAllMotors() {
        stopShootMotors();
        stopRotateMotor();
        stopAngleMotor();
    }

    public boolean getAngleForwardLimitSwitch() {
        return angleFwdLimit.isPressed();
    }

    public boolean getAngleReverseLimitSwitch() {
        return angleRevLimit.isPressed();
    }

    public void setTransfer(double output) {
        transferMotor.set(output);
    }

    public void setTransferMotorVelocity(double rpm) {
        transferMotor.getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    public void stopTransferMotor() {
        transferMotor.set(0);
    }

    public boolean getBallDetector(){
        return ballDetector.isPressed();
    }
}

