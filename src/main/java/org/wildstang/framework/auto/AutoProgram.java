package org.wildstang.framework.auto;

import java.util.ArrayList;
import java.util.List;

import org.wildstang.framework.auto.steps.control.AutoStepStopAutonomous;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.logger.Log;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a collection of AutoSteps used for a single Autonomous period.
 * @author Nathan
 */
public abstract class AutoProgram {

    protected final List<AutoStep> programSteps = new ArrayList<>();
    protected int currentStep;
    protected boolean finishedPreviousStep, finished, programStarted;

    /**
     * Use this method to set the steps for this program.
     * Programs execute the steps in the array programSteps serially.
     * Remember to clear everything before all of your steps are finished,
     * because once they are, it immediately drops into Sleeper.
     */
    protected abstract void defineSteps();

    /**
     * Collects the defined steps and starts the auto program.
     */
    public void initialize() {
        // prevent the steps from being double defined, AutoManager attempts to pre-define steps
        if (programSteps.isEmpty())
        {
            defineSteps();
        }
        loadStopPosition();
        currentStep = 0;
        finishedPreviousStep = false;
        finished = false;
        startStep(currentStep);
        programStarted = true;
    }

    /**
     * Remove all steps from the AutoProgram
     */
    public void cleanup() {
        programSteps.clear();
    }

    /**
     * Determines if the program's steps have been defined.
     * @return Returns true if there are any steps.
     */
    public boolean areStepsDefined() {
        return !programSteps.isEmpty();
    }

    /**
     * Update the current running step and move on if necessary.
     */
    public void update() {
        if (finished) {
            return;
        }
        if (finishedPreviousStep) {
            finishedPreviousStep = false;
            currentStep++;
            if (currentStep >= programSteps.size()) {
                finished = true;
                return;
            }
            else {
                startStep(currentStep);
                Log.warn(programSteps.get(currentStep).toString());
            }
        }
        if (programSteps.size() > currentStep && programStarted) {
            AutoStep step = programSteps.get(currentStep); // Prevent errors caused by mistyping.
            SmartDashboard.putString("Current auto step", step.toString());
            step.update();
            if (step.isFinished()) {
                Log.info("Finishing auto step: " + step.toString());
                finishedPreviousStep = true;
            }
        }
    }

    /**
     * Starts a given AutoStep and announces it at Info level.
     * @param stepIdx The index of the new program to run.
     */
    private void startStep(int stepIdx) {
        AutoStep step = programSteps.get(stepIdx);
        step.initialize();
        Log.info("Starting auto step: " + step.toString());
    }

    /**
     * Returns the current running AutoStep within the AutoProgram.
     * @return Current running AutoStep.
     */
    public AutoStep getCurrentStep() {
        return programSteps.get(currentStep);
    }

    /**
     * Returns the next AutoStep to run or null if at the last step.
     * @return Next AutoStep or null if no more steps.
     */
    public AutoStep getNextStep() {
        if (currentStep + 1 < programSteps.size()) {
            return programSteps.get(currentStep + 1);
        } else {
            return null;
        }
    }

    /**
     * Adds an AutoStepStopAutonomous after a given AutoStep index from config.
     * This allows insertion of a force stop of autonomous into a program via config.
     */
    protected void loadStopPosition() {
        int forceStopAtStep = Core.getConfigManager().getConfig()
                .getInt(this.getClass().getName() + ".ForceStopAtStep", 0);
        if (forceStopAtStep != 0) {
            int forceStop = forceStopAtStep;
            if ((forceStop <= programSteps.size()) && (forceStop > 0)) {
                programSteps.set(forceStop, new AutoStepStopAutonomous());
                Log.info("Force stopping auto step");
            } else {
                Log.warn("Force stop bounds exceeded");
            }
        }
    }

    /**
     * Returns true if the AutoProgram is complete.
     * @return True if the AutoProgram is complete.
     */
    public boolean isFinished() {
        return finished;
    }

    /**
     * Adds a new AutoStep to the AutoProgram.
     * @param newStep New AutoStep to add.
     */
    protected void addStep(AutoStep newStep) {
        programSteps.add(newStep);
    }

    /**
     * Returns the name of the AutoProgram.
     * @return Name of the AutoProgram.
     */
    @Override
    public abstract String toString();
}
