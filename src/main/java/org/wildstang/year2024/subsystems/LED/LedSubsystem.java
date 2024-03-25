package org.wildstang.year2024.subsystems.LED;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LedSubsystem implements Subsystem{

    public enum LedColor {FLASH_ORANGE, GREEN, SOLID_ORANGE, BLUE, PULSE_BLUE, PINK, YELLOW}

    public enum TieDye {NAVYBLUE, LIGHTBLUE, GRAY};
    public TieDye tieDyeColor;
    public static LedColor ledState;
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int ledIndex = 0;
    private int ledIndexFirst = 0;
    private int ledIndexLast = 0;
    

    private int port = 0;
    private int length = 53;

    boolean goBackward = false;

    private int value, adjust;

    Timer clock = new Timer();
    Timer clockTwo = new Timer();

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
            case PINK:
                for (int i = 0; i < length; i++) {
                    ledBuffer.setRGB(i, 255, 51, 255);
                }
                break;
            case YELLOW:
                for(int i = 0; i < length; i++){
                    ledBuffer.setRGB(i,255,255,51);
                }
            case GREEN:
                for (int i = 0; i < length; i++) {
                    ledBuffer.setRGB(i, 0, 255, 0);
                }
                break;
            case BLUE:

                //bouncingLaser();
                //blueTieDye();
                fadingSnake();
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

    public void fadingSnake(){
        clock.start(); 
        ledBuffer.setRGB(ledIndexFirst,0, 0, 150);
        
        if(clock.hasElapsed(0.02)){
            if((ledIndexFirst >= 0) && (ledIndexFirst < length-5)){
                ledIndexFirst++;
                ledBuffer.setRGB(ledIndexFirst-1,0,0,0);
                
               
                

            }
        clock.stop();
        clock.reset();
        }          
        int brightness = 140;
        int brightnessDecrement = 150 / (length-1);
        for(int i = ledIndexFirst; i >= 0; i--){
                ledBuffer.setRGB(i, 0, 0, brightness);
                brightness -= brightnessDecrement;
        }
        /*for(int i = length-1; i > ledIndexFirst; i--){
                    ledBuffer.setRGB(i, 0, 0, brightness);
                    brightness -= brightnessDecrement;
                }*/
    }          

    

    public void blueTieDye(){
        clock.start();
                if(clock.hasElapsed(0.05)){
                    for(int i = 0; i < length; i++){
                        int randomNum = (int)(Math.random()*2);
                        Timer timer = new Timer();
                        timer.start();
                        switch(randomNum){
                        case 0:
                            ledBuffer.setRGB(i, 0, 0, 153);
                            break;
                        case 1:
                            ledBuffer.setRGB(i, 102, 178, 150);
                            break;
                        case 2:
                            ledBuffer.setRGB(i, 200, 153, 200);
                            break;
                        }
                    }
                    clock.stop();
                    clock.reset();
                }
    }

    public void bouncingLaser(){
        
        clock.start();
                
        if(ledIndex == 0){
                    ledBuffer.setRGB(ledIndex, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+1, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+2, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+3, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+4, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+5, 0, 0, 150);
                    goBackward = false;
        }else if(ledIndex > 44){
                    ledBuffer.setRGB(ledIndex+1, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+2, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+3, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+4, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+5, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+6, 0, 0, 150);
                    goBackward = true;
                }else if(((ledIndex > 0) && (ledIndex < 48)) && goBackward){
                    ledBuffer.setRGB(ledIndex-1, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+1, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+2, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+3, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+4, 0, 0, 0);
                }else if(((ledIndex > 0) && (ledIndex < 48)) && !goBackward){
                    ledBuffer.setRGB(ledIndex-1, 0, 0, 0);
                    ledBuffer.setRGB(ledIndex, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+1, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+2, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+3, 0, 0, 150);
                    ledBuffer.setRGB(ledIndex+4, 0, 0, 150);
                }
                if(clock.hasElapsed(0.005)){
                    if(goBackward){
                        ledIndex--;
                    }else{
                        ledIndex++;
                    }
                    clock.stop();
                    clock.reset();
                }
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
