package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootSubsystem extends SubsystemBase {
    private final WPI_TalonFX shootMotor1 = new WPI_TalonFX(Constants.MOTOR_SHOOT_MAIN_1);
    private final WPI_TalonFX shootMotor2 = new WPI_TalonFX(Constants.MOTOR_SHOOT_MAIN_2);
    private final CANSparkMax triggerMotor = new CANSparkMax(Constants.MOTOR_SHOOT_TRIGGER, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax angleMotor = new CANSparkMax(Constants.MOTOR_SHOOT_ANGLE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rotateMotor = new CANSparkMax(Constants.MOTOR_SHOOT_ROTATE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SparkMaxLimitSwitch angleFwdLimit = angleMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    private final SparkMaxLimitSwitch angleRevLimit = angleMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

    public ShootSubsystem() {
        // Reset motor controllers to factory defaults
        shootMotor1.configFactoryDefault();
        shootMotor2.configFactoryDefault();
        rotateMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();
        triggerMotor.restoreFactoryDefaults();

        // Configure invert on motors
        shootMotor1.setInverted(Constants.MOTOR_SHOOT_1_INVERTED);
        shootMotor2.setInverted(Constants.MOTOR_SHOOT_2_INVERTED);
        rotateMotor.setInverted(Constants.MOTOR_SHOOT_ROTATE_INVERTED);
        angleMotor.setInverted(Constants.MOTOR_SHOOT_ANGLE_INVERTED);
        triggerMotor.setInverted(Constants.MOTOR_SHOOT_TRIGGER_INVERTED);

        // Configure neutral/idle mode on motors
        shootMotor1.setNeutralMode(NeutralMode.Coast);
        shootMotor2.setNeutralMode(NeutralMode.Coast);
        rotateMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        triggerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Configure current limit on motors
        shootMotor1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 38, 0, 0));
        shootMotor2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 38, 0, 0));
        rotateMotor.setSmartCurrentLimit(30);
        angleMotor.setSmartCurrentLimit(30);
        triggerMotor.setSmartCurrentLimit(30);

        // Configure PID controller on motors
        shootMotor1.config_kP(0, Constants.PID_SHOOT_MAIN[0]);
        shootMotor1.config_kI(0, Constants.PID_SHOOT_MAIN[1]);
        shootMotor1.config_kD(0, Constants.PID_SHOOT_MAIN[2]);
        shootMotor1.config_kF(0, Constants.FF_SHOOT_MAIN);
        shootMotor2.config_kP(0, Constants.PID_SHOOT_MAIN[0]);
        shootMotor2.config_kI(0, Constants.PID_SHOOT_MAIN[1]);
        shootMotor2.config_kD(0, Constants.PID_SHOOT_MAIN[2]);
        shootMotor2.config_kF(0, Constants.FF_SHOOT_MAIN);


        // set conversion factor by gearing
        double rotateFactor = 1/Constants.SHOOT_ROTATE_GEARING;
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
        rotatePIDController.setP(Constants.PID_SHOOT_ROTATE[0]);
        rotatePIDController.setI(Constants.PID_SHOOT_ROTATE[1]);
        rotatePIDController.setD(Constants.PID_SHOOT_ROTATE[2]);
        rotatePIDController.setFF(Constants.FF_SHOOT_ROTATE);
        rotatePIDController.setSmartMotionMaxVelocity(Constants.SMART_MOTION_MAX_VELOCITY_SHOOT_ROTATE, 0);
        rotatePIDController.setSmartMotionMaxAccel(Constants.SMART_MOTION_MAX_ACCEL_SHOOT_ROTATE, 0);

        SparkMaxPIDController triggerPIDController = triggerMotor.getPIDController();
        triggerPIDController.setP(Constants.PID_SHOOT_TRIGGER[0]);
        triggerPIDController.setI(Constants.PID_SHOOT_TRIGGER[1]);
        triggerPIDController.setD(Constants.PID_SHOOT_TRIGGER[2]);
        triggerPIDController.setFF(Constants.FF_SHOOT_TRIGGER);

        rotateMotor.set(0);
        rotateMotor.getEncoder().setPosition(0);

        // Enable hard limit switch
        angleFwdLimit.enableLimitSwitch(true);
        angleRevLimit.enableLimitSwitch(true);
        // Enable soft limit switch
        angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.SOFT_LIMIT_FWD_SHOOT_ANGLE);
        angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

        angleMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        if (angleRevLimit.isPressed()){
            angleMotor.getEncoder().setPosition(0);
        }
    }

    public void setShootMotorsVelocity(double rpm){
        shootMotor1.set(ControlMode.Velocity, rpm * 2048.0 / 600.0);
        shootMotor2.set(ControlMode.Velocity, rpm * 2048.0 / 600.0);
    }

    public double getShootMotorsVelocity(){
        return (shootMotor1.getSelectedSensorVelocity() + shootMotor2.getSelectedSensorVelocity())/2.0;
    }

    public void stopShootMotors(){
        shootMotor1.set(ControlMode.PercentOutput, 0);
        shootMotor2.set(ControlMode.PercentOutput, 0);
    }

    public void setRotateMotorPosition(double position){
        while (position>0.85){
            position-=1;
        }
        while (position<-0.15){
            position+=1;
        }
        if (position>=0.7&&position<=0.85){
            rotateMotor.set(0);
            return;
        }
        rotateMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kSmartMotion);
    }

    public double getRotateMotorPosition(){
        return rotateMotor.getEncoder().getPosition();
    }

    public void stopRotateMotor(){
        rotateMotor.stopMotor();
    }

    public void setAngleMotorPosition(double position){
        angleMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kSmartMotion);
    }

    public void setAngleMotorOutput(double output){
        angleMotor.getPIDController().setReference(output, CANSparkMax.ControlType.kDutyCycle);
    }

    public double getAngleMotorPosition(){
        return angleMotor.getEncoder().getPosition();
    }

    public void stopAngleMotor(){
        angleMotor.stopMotor();
    }

    public void stopAllMotors(){
        stopShootMotors();
        stopRotateMotor();
        stopAngleMotor();
    }

    public boolean getAngleForwardLimitSwitch(){
        return angleFwdLimit.isPressed();
    }

    public boolean getAngleReverseLimitSwitch(){
        return angleRevLimit.isPressed();
    }

    public void setTrigger(double output){
        triggerMotor.set(output);
    }

    public void setTriggerMotorVelocity(double rpm){
        triggerMotor.getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    public void stopTriggerMotor(){
        triggerMotor.set(0);
    }
}

