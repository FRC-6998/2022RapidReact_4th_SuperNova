package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectSubsystem extends SubsystemBase {
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_INTAKE_FORWARD, Constants.SOLENOID_INTAKE_REVERSE);
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.MOTOR_INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);
    //private final CANSparkMax testMotor = new CANSparkMax(Constants.MOTOR_SHOOT_TRIGGER, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private boolean intakeEnabled = false;
    private final Timer timer = new Timer();
    public CollectSubsystem() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(30);
        intakeMotor.setInverted(Constants.MOTOR_INTAKE_INVERTED);
    }

    public void enableIntake(boolean reverse){
        solenoid.set(DoubleSolenoid.Value.kForward);
        double speed = reverse?-1:1;
        intakeMotor.set(speed);
        //testMotor.set(-speed);
        intakeEnabled = true;
    }
    public void disableIntake(){
        solenoid.set(DoubleSolenoid.Value.kOff);
        intakeMotor.set(0);
        //testMotor.set(0);
        intakeEnabled = false;
    }
    public void reverseIntake(){
        timer.reset();
        timer.start();
        solenoid.set(DoubleSolenoid.Value.kReverse);
        intakeMotor.set(1);
        if (timer.hasElapsed(0.1)){
            intakeMotor.set(0);
        }
    }

    public boolean isIntakeEnabled() {
        return intakeEnabled;
    }
}

