package org.firstinspires.ftc.teamcode.globals;

import java.util.HashMap;
import java.util.Map;

public enum Levels {
    INSTANCE;

    Levels (){
        levels.put(Levels.TSELocation.NONE,0);

        levels.put(Levels.TSELocation.LEVEL_1,1);
        levels.put(Levels.TSELocation.LEVEL_2,2);
        levels.put(Levels.TSELocation.LEVEL_3,3);

        Levels.getInstance().setTSELocation(Levels.TSELocation.LEVEL_3);
    }

    public enum TSELocation {
        LEVEL_3,
        LEVEL_2,
        LEVEL_1,
        NONE
    }

    Map<TSELocation, Integer> levels = new HashMap<>();

    private TSELocation selectedTSELocation = TSELocation.LEVEL_3;

    public void setTSELocation(TSELocation location){
        selectedTSELocation = location;
    }

    public TSELocation getTSELocation(){
        return selectedTSELocation;
    }

    public int getTSELevel(){

        return levels.get(selectedTSELocation);
    }

    public static Levels getInstance(){
        return INSTANCE;
    }
}
