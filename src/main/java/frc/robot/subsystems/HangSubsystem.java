package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangSubsystem extends SubsystemBase {
    private final CANSparkMax leftHangMotor = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightHangMotor = new CANSparkMax(21, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_HANG_FORWARD, Constants.SOLENOID_HANG_REVERSE);

    public HangSubsystem() {
        // Factory reset motors
        leftHangMotor.restoreFactoryDefaults();
        rightHangMotor.restoreFactoryDefaults();
        // Set motor's invert flag
        leftHangMotor.setInverted(Constants.MOTOR_HANG_LEFT_INVERTED);
        rightHangMotor.setInverted(Constants.MOTOR_HANG_RIGHT_INVERTED);
        // Set motor's current draw limit
        leftHangMotor.setSmartCurrentLimit(39);
        rightHangMotor.setSmartCurrentLimit(39);


        SparkMaxPIDController leftHangPIDController = leftHangMotor.getPIDController();
        leftHangPIDController.setP(Constants.PID_HANG[0]);
        leftHangPIDController.setI(Constants.PID_HANG[1]);
        leftHangPIDController.setD(Constants.PID_HANG[2]);
        leftHangPIDController.setFF(Constants.FF_HANG);
        leftHangPIDController.setSmartMotionMaxVelocity(Constants.SMART_MOTION_MAX_VELOCITY_HANG, 0);
        leftHangPIDController.setSmartMotionMaxAccel(Constants.SMART_MOTION_MAX_ACCEL_HANG, 0);
        SparkMaxPIDController rightHangPIDController = rightHangMotor.getPIDController();
        rightHangPIDController.setP(Constants.PID_HANG[0]);
        rightHangPIDController.setI(Constants.PID_HANG[1]);
        rightHangPIDController.setD(Constants.PID_HANG[2]);
        rightHangPIDController.setFF(Constants.FF_HANG);
        rightHangPIDController.setSmartMotionMaxVelocity(Constants.SMART_MOTION_MAX_VELOCITY_HANG, 0);
        rightHangPIDController.setSmartMotionMaxAccel(Constants.SMART_MOTION_MAX_ACCEL_HANG, 0);

        resetEncoder();

        leftHangMotor.set(0);
        rightHangMotor.set(0);
    }

    @Override
    public void periodic() {
        if (Constants.DEBUG) {
            SmartDashboard.putNumber("New HangSubsystem Left Setpoint", leftHangMotor.get());
            SmartDashboard.putNumber("New HangSubsystem Right Setpoint", rightHangMotor.get());
            SmartDashboard.putNumber("New HangSubsystem Left Position", leftHangMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("New HangSubsystem Right Position", rightHangMotor.getEncoder().getPosition());
        }
    }

    public void resetEncoder(){
        leftHangMotor.getEncoder().setPosition(0);
        rightHangMotor.getEncoder().setPosition(0);
    }

    public void set(double speed){
        leftHangMotor.set(speed);
        rightHangMotor.set(speed);
    }

    public void setSmartMotionSetpoint(double setpoint){
        leftHangMotor.getPIDController().setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
        rightHangMotor.getPIDController().setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    public void stopMotors(){
        leftHangMotor.set(0);
        rightHangMotor.set(0);
    }


    public void extendSolenoid(){
        doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void stopSolenoid(){
        doubleSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void collapseSolenoid(){
        doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean getSolenoid(){
        return doubleSolenoid.get() == DoubleSolenoid.Value.kForward;
    }
}