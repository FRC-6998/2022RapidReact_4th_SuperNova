package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectSubsystem extends SubsystemBase {
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_INTAKE_FORWARD, Constants.SOLENOID_INTAKE_REVERSE);
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.MOTOR_INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private boolean intakeEnabled = false;
    public CollectSubsystem() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(30);
        intakeMotor.setInverted(Constants.MOTOR_INTAKE_INVERTED);

        intakeMotor.getEncoder().setPositionConversionFactor(0.1);
        intakeMotor.getEncoder().setVelocityConversionFactor(0.1);

        SparkMaxPIDController intakePIDController = intakeMotor.getPIDController();
        intakePIDController.setP(Constants.PID_INTAKE[0]);
        intakePIDController.setI(Constants.PID_INTAKE[1]);
        intakePIDController.setD(Constants.PID_INTAKE[2]);
        intakePIDController.setFF(Constants.FF_INTAKE);
    }

    public void enableIntake(boolean reverse){
        solenoid.set(DoubleSolenoid.Value.kForward);
        double rpm = reverse?-1000:1000;
        intakeMotor.getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
        //testMotor.set(-speed);
        intakeEnabled = true;
    }
    public void disableIntake(){
        solenoid.set(DoubleSolenoid.Value.kReverse);
        intakeMotor.set(0);
        //testMotor.set(0);
        intakeEnabled = false;
    }

    public boolean isIntakeEnabled() {
        return intakeEnabled;
    }

    public void enableIntake() {
        enableIntake(false);
    }
}

