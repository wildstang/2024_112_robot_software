package org.wildstang.year2024.robot;

import org.wildstang.framework.core.AutoPrograms;
import org.wildstang.year2024.auto.Programs.ElimsOneA;
import org.wildstang.year2024.auto.Programs.ElimsOneB;
import org.wildstang.year2024.auto.Programs.ElimsTwo;
import org.wildstang.year2024.auto.Programs.OneNoteWingAutoTop;
import org.wildstang.year2024.auto.Programs.PreloadAuto;
import org.wildstang.year2024.auto.Programs.QualsAuto;
import org.wildstang.year2024.auto.Programs.QualsPlus;
import org.wildstang.year2024.auto.Programs.WingABC_Amp;
import org.wildstang.year2024.auto.Programs.WingAB_Amp;
import org.wildstang.year2024.auto.Programs.WingA_Amp;
import org.wildstang.year2024.auto.Programs.WingBAC_Speaker;
import org.wildstang.year2024.auto.Programs.WingBA_Speaker;
import org.wildstang.year2024.auto.Programs.WingB_Speaker;

/**
 * All active AutoPrograms are enumerated here.
 * It is used in Robot.java to initialize all programs.
 */
public enum WsAutoPrograms implements AutoPrograms {

    // enumerate programs
    ELIMS_1A("ElimsOneA", ElimsOneA.class),
    ELIMS_1B("ElimsOne 1b", ElimsOneB.class),
    ELIMS_2("ElimsTwo", ElimsTwo.class),
    ONE_NOTE_WING_AUTO_TOP("One Note Wing Auto Top", OneNoteWingAutoTop.class),
    PRELOAD_AUTO("Preload Auto", PreloadAuto.class),
    QUALS_AUTO("Quals Auto", QualsAuto.class),
    QUALS_PLUS("Quals Plus", QualsPlus.class),
    WINGA_AMP("WingA Amp", WingA_Amp.class),
    WINGAB_AMP("WingAB Amp", WingAB_Amp.class),
    WINGABC_AMP("WingABC Amp", WingABC_Amp.class),
    WINGB_SPEAKER("WingB Speaker", WingB_Speaker.class),
    WINGBA_SPEAKER("WingBA Speaker", WingBA_Speaker.class),
    WINGBAC_SPEAKER("WingBAC Speaker", WingBAC_Speaker.class),

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