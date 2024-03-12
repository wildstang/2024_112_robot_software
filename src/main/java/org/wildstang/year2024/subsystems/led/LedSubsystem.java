package org.wildstang.year2024.subsystems.led;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LedSubsystem implements Subsystem{

    public enum LedColor {FLASH_ORANGE, GREEN, SOLID_ORANGE, BLUE}
    public static LedColor ledState;
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private int port = 0;
    private int length = 53;

    Timer clock = new Timer();

    @Override
    public void inputUpdate(Input source) {
        
    }

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
    public void selfTest() {
        
    }

    @Override
    public void update() {

        clock.start();
        switch(ledState){
            case SOLID_ORANGE:
                for (int i = 0; i < length; i++){
                    ledBuffer.setRGB(i, 255, 140, 0);
                }
                break;
            case FLASH_ORANGE:
                if(clock.hasElapsed(0.25)){
                    for (int i = 0; i < length; i++){
                        ledBuffer.setRGB(i, 255, 140, 0);
                    }
                    
                }
                if(clock.hasElapsed(0.5)){
                    for (int i = 0; i < length; i++){
                        ledBuffer.setRGB(i, 255, 255, 255);
                    }
                    clock.reset();
                }
                break;
            case GREEN:
                for (int i = 0; i < length; i++){
                    ledBuffer.setRGB(i, 0, 255, 0);
                }
                break;
            case BLUE:
            default:
                for (int i = 0; i < length; i++){
                    ledBuffer.setRGB(i, 0, 0, 255);
                }
                break;

        }   

        clock.reset();
        led.setData(ledBuffer);
        led.start();
        SmartDashboard.putString("led state", ledState.name());
    }

    @Override
    public void resetState() {
        ledState = LedColor.BLUE;
     
    }

    @Override
    public String getName() {
        return "Led Subsystem";
    }

    public void setColorRGB(int red, int green, int blue){
        for (int i = 0; i < length; i++){
            ledBuffer.setRGB(i, red, green, blue);
        }
    }
    
}
