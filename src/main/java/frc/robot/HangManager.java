package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.hang.ExtendHangCommand;
import frc.robot.commands.hang.MoveToHangSetpointCommand;
import frc.robot.commands.shoot.RotateToAngleCommand;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class HangManager {
    private final HangSubsystem hangSubsystem;
    private final ShootSubsystem shootSubsystem;
    private int step = -1;

    public HangManager(HangSubsystem hangSubsystem, ShootSubsystem shootSubsystem){
        this.hangSubsystem = hangSubsystem;
        this.shootSubsystem = shootSubsystem;
    }

    public void setStep(int step){
        this.step = step;
    }

    public int getStep() {
        return step;
    }

    public SequentialCommandGroup nextCommand(){
        switch (step){
            case -1:
                step = 0;
                return new SequentialCommandGroup(
                        new RotateToAngleCommand(shootSubsystem, 270),
                        new ExtendHangCommand(hangSubsystem)
                );
            case 0:
                step = 1;
                // go to 36° from vertical
                return new SequentialCommandGroup(
                        new MoveToHangSetpointCommand(hangSubsystem, 80)
                );
            case 1:
                step = 2;
                // go to 198° from vertical (mid+high)
                return new SequentialCommandGroup(
                        new MoveToHangSetpointCommand(hangSubsystem, 173),
                        new MoveToHangSetpointCommand(hangSubsystem, 160)
                );
            case 2:
                step = 4;
                // go to 117° to unlock hook and then go to 360° from vertical (high+traversal)
                return new SequentialCommandGroup(
                        new MoveToHangSetpointCommand(hangSubsystem, 125),
                        new DelayCommand(1),
                        new MoveToHangSetpointCommand(hangSubsystem, 160),
                        new MoveToHangSetpointCommand(hangSubsystem, 264),
                        new MoveToHangSetpointCommand(hangSubsystem, 254)
                );
            case 3:
                step = 4;
                // go to 360° from vertical (high+traversal)
                return new SequentialCommandGroup(
                        new MoveToHangSetpointCommand(hangSubsystem, 264),
                        new MoveToHangSetpointCommand(hangSubsystem, 254)
                );
            case 4:
                step = 6;
                // go to 297° to unlock hook from high rung and then go to 360° from vertical (traversal)
                return new SequentialCommandGroup(
                        new MoveToHangSetpointCommand(hangSubsystem, 215),
                        new DelayCommand(1),
                        new MoveToHangSetpointCommand(hangSubsystem, 260)
                );
            case 5:
                step = 6;
                // go to 360° from vertical (traversal)
                return new SequentialCommandGroup(
                        new MoveToHangSetpointCommand(hangSubsystem, 260)
                );
            default:
                break;
        }
        return new SequentialCommandGroup();
    }

    public SequentialCommandGroup prevCommand(){
        switch (step){
            case 1:
                step = 0;
                return new SequentialCommandGroup(
                        new MoveToHangSetpointCommand(hangSubsystem, 60)
                );
            case 2:
                return new SequentialCommandGroup(
                        new MoveToHangSetpointCommand(hangSubsystem, 125)
                );
            default:
                break;
        }
        return new SequentialCommandGroup();
    }
}
