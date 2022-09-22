package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes;

import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.AbstractAutonState;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.StateConfigureAutonRoutine;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.StateDetectStarterStack;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.StateInitialize;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.StateMoveToLaunchLine;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.StateMoveToTargetZone;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.StateParkOnLaunchLine;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.StatePlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.StatePrematchSetup;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.StateShootPowerShots;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.StateUnfoldCrane;

import java.util.ArrayList;
import java.util.Arrays;

public enum PresetAutonRoutine {

    STANDARD(new ArrayList<>(Arrays.asList(
            StateConfigureAutonRoutine.class,
            StatePrematchSetup.class,
            StateDetectStarterStack.class,
            StateInitialize.class,
            StateMoveToTargetZone.class,
            StateUnfoldCrane.class,
            StatePlaceWobbleGoal.class,
            StateMoveToLaunchLine.class,
            StateShootPowerShots.class,
            StateParkOnLaunchLine.class))),

    AGGRESSIVE(new ArrayList<>(Arrays.asList(
            StateConfigureAutonRoutine.class,
            StatePrematchSetup.class,
            StateDetectStarterStack.class,
            StateInitialize.class,
            StateMoveToTargetZone.class,
            StateUnfoldCrane.class,
            StatePlaceWobbleGoal.class,
            StateMoveToLaunchLine.class,
            StateShootPowerShots.class)));

    private ArrayList<Class<? extends AbstractAutonState>> sequence;

    PresetAutonRoutine(ArrayList<Class<? extends AbstractAutonState>> sequenceIn)
    {
        this.sequence = sequenceIn;
    }

    public ArrayList<Class<? extends AbstractAutonState>>getSequence(){
        // Get the preset list
        return this.sequence;
    }

    PresetAutonRoutine(){

    }
}

