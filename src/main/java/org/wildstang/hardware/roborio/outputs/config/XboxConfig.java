package org.wildstang.hardware.roborio.outputs.config;

import org.wildstang.framework.hardware.OutputConfig;

/**
 * Contains configurations for servos.
 */
public class XboxConfig implements OutputConfig {

    private int m_channel = 0;
    private double m_default;

    /**
     * Construct the servo config.
     * @param channel Hardware port number.
     * @param p_default Default position.
     */
    public XboxConfig(int channel) {
        m_channel = channel;
    }

    /**
     * Returns the hardware port number.
     * @return The hardware port number.
     */
    public int getChannel() {
        return m_channel;
    }

    /**
     * Returns the default position.
     * @return The default position.
     */
    public double getDefault() {
        return m_default;
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