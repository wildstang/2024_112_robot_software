package org.wildstang.hardware.roborio.outputs;

import org.wildstang.framework.io.outputs.AnalogOutput;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Controls a servo.
 */
public class XboxControllerOutput extends AnalogOutput {
    XboxController joystick;
    /**
     * Constructs the servo from config.
     * @param name Descriptive name of the servo.
     * @param channel Hardware port number the servo is connected to.
     * @param p_default Default position.
     */

    public XboxControllerOutput(String p_name) {
        super(p_name);
        this.joystick = new XboxController(0);
        
    }

    /**
     * Sets servo position to current value.
     */
    @Override
    public void sendDataToOutput() {
        joystick.setRumble(GenericHID.RumbleType.kBothRumble, 1); // or kLeftRumble/kRightRumble
    }

    /**
     * Does nothing, config values only affects start state.
     */
    public void notifyConfigChange() { }
}