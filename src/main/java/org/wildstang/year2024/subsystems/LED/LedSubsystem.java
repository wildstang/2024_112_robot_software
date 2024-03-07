package org.wildstang.year2024.subsystems.LED;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LedSubsystem implements Subsystem{

    public enum LEDColor {FLASH_ORANGE, GREEN, SOLID_ORANGE}
    public static LEDColor ledState;
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private int port = 0;
    private int length = 15;

    Timer clock = new Timer();

    @Override
    public void inputUpdate(Input source) {
        
    }

    @Override
    public void init() {
        led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);
        led.setLength(ledBuffer.getLength());
        for (int i = 0; i < length; i++){
            ledBuffer.setRGB(i, 255, 255, 255);
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
        }   

        clock.reset();
        led.setData(ledBuffer);
        led.start();
    }

    @Override
    public void resetState() {
     
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
