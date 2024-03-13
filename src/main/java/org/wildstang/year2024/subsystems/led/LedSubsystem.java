package org.wildstang.year2024.subsystems.LED;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LedSubsystem implements Subsystem{

    public enum LedColor {FLASH_ORANGE, GREEN, SOLID_ORANGE, BLUE, PULSE_BLUE, YELLOW}
    public static LedColor ledState;
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private int port = 0;
    private int length = 53;

    private int value, adjust;

    Timer clock = new Timer();

    @Override
    public void init() {
        ledState = LedColor.BLUE;
        led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);
        led.setLength(ledBuffer.getLength());
        for (int i = 0; i < length; i++){
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        led.setData(ledBuffer);
        led.start();
        resetState();
    }

    @Override
    public void inputUpdate(Input source) {
        
    }

    @Override
    public void update() {
        switch(ledState){
            case SOLID_ORANGE:
                for (int i = 0; i < length; i++) {
                    ledBuffer.setRGB(i, 255, 140, 0);
                }
                break;
            case FLASH_ORANGE:
                if(value == 255) {
                    for (int i = 0; i < length; i++) {
                        ledBuffer.setRGB(i, 255, 140, 0);
                    }
                    
                } else if(value == 0) {
                    for (int i = 0; i < length; i++) {
                        ledBuffer.setRGB(i, 255, 255, 255);
                    }
                }
                break;
            case YELLOW:
                for (int i = 0; i < length; i++) {
                    ledBuffer.setRGB(i, 255, 255, 0);
                }
                break;
            case GREEN:
                for (int i = 0; i < length; i++) {
                    ledBuffer.setRGB(i, 0, 255, 0);
                }
                break;
            case BLUE:
                for (int i = 0; i < length; i++) {
                    ledBuffer.setRGB(i, 0, 0, 255);
                }
                break;
            case PULSE_BLUE:
                for (int i = 0; i < length; i++) {
                    ledBuffer.setRGB(i, 0, 0, value);
                }
                break;
        }  

        value += adjust;
        if (value >= 255) {
            value = 255;
            adjust *= -1;
        } else if (value <= 0) {
            value = 0;
            adjust *= -1;
        }

        led.setData(ledBuffer);
        led.start();
        SmartDashboard.putString("led state", ledState.name());
    }

    @Override
    public void selfTest() {
    } 

    @Override
    public void resetState() {
        ledState = LedColor.BLUE;
        value = 0;
        adjust = 5;
    }

    @Override
    public String getName() {
        return "Led Subsystem";
    }
}
