package org.wildstang.hardware.roborio.outputs.config;

import org.wildstang.framework.hardware.OutputConfig;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Contains configurations for servos.
 */
public class WsXboxControllerConfig implements OutputConfig {

    private int m_channel = 0;
    private final RumbleType m_bothMotors;

    /**
     * Construct the servo config.
     * @param channel Hardware port number.
     * @param p_default Default position.
     */
    public WsXboxControllerConfig(int channel, RumbleType bothMotors) {
        m_channel = channel;
        m_bothMotors = bothMotors;
    }

    /**
     * Returns the hardware port number.
     * @return The hardware port number.
     */
    public int getChannel() {
        return m_channel;
    }

    public RumbleType getbothMotors() {
        return m_bothMotors;
    }



    /**
     * Builds a JSON String describing the digital output config.
     * @return Port number.
     */
    @Override
    public String toString() {
        StringBuffer buf = new StringBuffer();
        buf.append("{\"channel\": ");
        buf.append(m_channel);
        buf.append("}");
        return buf.toString();
    }

}