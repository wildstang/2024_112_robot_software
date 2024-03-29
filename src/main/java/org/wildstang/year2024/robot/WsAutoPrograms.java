package org.wildstang.year2024.robot;

import org.wildstang.framework.core.AutoPrograms;
import org.wildstang.year2024.auto.Programs.ElimsOneA;
import org.wildstang.year2024.auto.Programs.ElimsTwo;
import org.wildstang.year2024.auto.Programs.OneNoteWingAutoTop;
import org.wildstang.year2024.auto.Programs.PreloadAuto;
import org.wildstang.year2024.auto.Programs.QualsAuto;
import org.wildstang.year2024.auto.Programs.QualsPlus;
import org.wildstang.year2024.auto.Programs.WingA_Amp;

/**
 * All active AutoPrograms are enumerated here.
 * It is used in Robot.java to initialize all programs.
 */
public enum WsAutoPrograms implements AutoPrograms {

    // enumerate programs
    //SAMPLE_PROGRAM("Sample", SampleAutoProgram.class),
    //TEST_PROGRAM("Test Program", Testprogram.class),
    ELIMS_ONE("ElimsOne", ElimsOneA.class),
    // ELIMS_TWO("Elims Two", ElimsTwo.class),
    // FULL_AUTO_BOTTOM("Full Auto Bottom", FullAutoBottom.class),
    // FULL_AUTO_TOP("Full Auto Top", FullAutoTop.class),
    // FULL_WING_AUTO_TOP("Full Wing Auto Top", FullWingAutoTop.class),
    // ONE_NOTE_WING_AUTO_TOP("One Note Wing Auto Top", OneNoteWingAutoTop.class),
    // QUALS_AUTO("Quals Auto", QualsAuto.class),
    // QUALS_PLUS("Quals Plus", QualsPlus.class),
    PRELOAD_AUTO("Preload Auto", PreloadAuto.class),
    WINGA_AMP("WingA Amp", WingA_Amp.class),
    // TWO_NOTE_WING_AUTO_TOP("Two Note Wing Auto Top", TwoNoteWingAutoTop.class),
    // ROCKET_AUTO_TOP("Rocket Auto Top", RocketAutoTop.class),

    ;

    /**
     * Do not modify below code, provides template for enumerations.
     * We would like to have a super class for this structure, however,
     * Java does not support enums extending classes.
     */
    
    private String name;
    private Class<?> programClass;

    /**
     * Initialize name and AutoProgram map.
     * @param name Name, must match that in class to prevent errors.
     * @param programClass Class containing AutoProgram
     */
    WsAutoPrograms(String name, Class<?> programClass) {
        this.name = name;
        this.programClass = programClass;
    }

    /**
     * Returns the name mapped to the AutoProgram.
     * @return Name mapped to the AutoProgram.
     */
    @Override
    public String getName() {
        return name;
    }

    /**
     * Returns AutoProgram's class.
     * @return AutoProgram's class.
     */
    @Override
    public Class<?> getProgramClass() {
        return programClass;
    }
}