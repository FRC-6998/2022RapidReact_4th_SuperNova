package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {
    public enum Effect{
        FILL, CYCLE, FLASH, BREATH, CUSTOM
    }

    private Effect effect = Effect.CUSTOM;
    private final AddressableLED led = new AddressableLED(Constants.LED_PWM_PORT);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LED_LENGTH);

    public LedSubsystem() {
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    @Override
    public void periodic() {
        switch (effect){
            case CYCLE:
                break;
            default:
                break;
        }
    }

    public void setRGB(int index, Color color) {
        effect = Effect.CUSTOM;
        ledBuffer.setLED(index, color);
    }

    public void setRGB(int index, int r, int g, int b) {
        effect = Effect.CUSTOM;
        ledBuffer.setRGB(index, r, g, b);
    }

    public void setHSV(int index, int h, int s, int v) {
        effect = Effect.CUSTOM;
        ledBuffer.setHSV(index, h, s, v);
    }

    public void write() {
        led.setData(ledBuffer);
    }

    public void close() {
        led.close();
    }

    public void fillRGB(Color color) {
        effect = Effect.FILL;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            setRGB(i, color);
        }
    }

    public void fillRGB(int r, int g, int b) {
        effect = Effect.FILL;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            setRGB(i, r, g, b);
        }
    }

    public void fillHSV(int h, int s, int v) {
        effect = Effect.FILL;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            setHSV(i, h, s, v);
        }
    }
}

