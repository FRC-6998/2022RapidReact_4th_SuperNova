package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class HangSubsystem extends SubsystemBase {
    private final CANSparkMax leftHangMotor = new CANSparkMax(Constants.MOTOR_HANG_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightHangMotor = new CANSparkMax(Constants.MOTOR_HANG_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_HANG_FORWARD, Constants.SOLENOID_HANG_REVERSE);
    private boolean hangEnable = false;
    private Timer timer = new Timer();

    public HangSubsystem() {
        // Factory reset motors
        leftHangMotor.restoreFactoryDefaults();
        rightHangMotor.restoreFactoryDefaults();
        // Set motor's invert flag
        leftHangMotor.setInverted(Constants.MOTOR_HANG_LEFT_INVERTED);
        rightHangMotor.setInverted(Constants.MOTOR_HANG_RIGHT_INVERTED);

        leftHangMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightHangMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
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

        rightHangMotor.follow(leftHangMotor, true);

        leftHangMotor.set(0);
        rightHangMotor.set(0);

        hangEnable = false;
        timer.reset();
    }

    @Override
    public void periodic() {
        if (Constants.DEBUG) {
            SmartDashboard.putNumber("New HangSubsystem Left Setpoint", leftHangMotor.get());
            SmartDashboard.putNumber("New HangSubsystem Right Setpoint", rightHangMotor.get());
            SmartDashboard.putNumber("New HangSubsystem Left Position", leftHangMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("New HangSubsystem Right Position", rightHangMotor.getEncoder().getPosition());
        }
        if (timer.hasElapsed(90)){
            hangEnable = true;
            timer.stop();
        }

        SmartDashboard.putBoolean("Hang Enable",hangEnable);
    }

    public void resetEncoder(){
        leftHangMotor.getEncoder().setPosition(0);
        rightHangMotor.getEncoder().setPosition(0);
    }

    public void set(double speed){
        leftHangMotor.set(speed);
        //rightHangMotor.set(speed);
    }

    public void setSmartMotionSetpoint(double setpoint){
        leftHangMotor.getPIDController().setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
        //rightHangMotor.getPIDController().setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    public void stopMotors(){
        leftHangMotor.set(0);
        //rightHangMotor.set(0);
    }

    public double getHangerPosition(){
        return (leftHangMotor.getEncoder().getPosition() + rightHangMotor.getEncoder().getPosition())/2.0;
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
        return doubleSolenoid.get() == DoubleSolenoid.Value.kForward || doubleSolenoid.get() == DoubleSolenoid.Value.kOff;
    }

    public void setIdleMode(CANSparkMax.IdleMode mode){
        leftHangMotor.setIdleMode(mode);
        rightHangMotor.setIdleMode(mode);
    }

    public boolean getHangerEnable(){
        return hangEnable;
    }
    public void setHangerEnable(boolean enable){
        hangEnable = enable;
    }
    public void startTimer(){
        timer.start();
    }
    public void resetTimer(){
        timer.reset();
    }
}