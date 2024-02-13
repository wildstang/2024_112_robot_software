package org.wildstang.year2024.subsystems.targeting;

// ton of imports
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.framework.io.inputs.Input;

public class WsVision implements Subsystem {

    public LimeConsts LC;

    @Override
    public void inputUpdate(Input source) {
    }

    @Override
    public void init() {
        LC = new LimeConsts();

        resetState();
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
    }

    @Override
    public void resetState() {
    }

    @Override
    public String getName() {
        return "Ws Vision";
    }
}