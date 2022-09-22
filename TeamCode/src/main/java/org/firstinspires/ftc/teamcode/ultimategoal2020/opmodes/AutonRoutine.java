package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes;

import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.AbstractAutonState;

import java.util.ArrayList;

public class AutonRoutine {
    private ArrayList<Class<? extends AbstractAutonState>> sequence;

    public ArrayList<Class<? extends AbstractAutonState>> getSequence(){
        return this.sequence;
    }

    public AutonRoutine (PresetAutonRoutine presetRoutine){
        this.sequence = new ArrayList<>(presetRoutine.getSequence());
    }

    public Class<? extends AbstractAutonState> getNextAutonStateClass(){
        Class<? extends AbstractAutonState> nextStateClass = null;
        try{
            // The remove method returns the removed item
            nextStateClass = this.sequence.remove(0);
        } catch (Exception e){
            System.out.println("In catch of AutonSequence::getnextAutonStateClass");
            System.out.println(e.getMessage());
        }
        return nextStateClass;
    }
}
