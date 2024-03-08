package org.wildstang.hardware.roborio.outputs;

import org.wildstang.framework.io.outputs.AnalogOutput;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Controls a servo.
 */
public class XboxControllerOutput extends AnalogOutput {
    XboxController joystick;
    RumbleType curRumbleType;
    int channel;
    
    /**
     * Constructs the servo from config.
     * @param name Descriptive name of the servo.
     * @param channel Hardware port number the servo is connected to.
     * @param p_default Default position.
     */

    public XboxControllerOutput(String p_name, int channel, RumbleType rumble) {
        super(p_name);
        curRumbleType = rumble;
        this.channel = channel;
        this.joystick = new XboxController(channel);
        
    }

    /**
     * Sets servo position to current value.
     */
    @Override
    public void sendDataToOutput() {
        joystick.setRumble(curRumbleType, getValue()); // or kLeftRumble/kRightRumble
    }

    /**
     * Does nothing, config values only affects start state.
     */
    public void notifyConfigChange() { }
}