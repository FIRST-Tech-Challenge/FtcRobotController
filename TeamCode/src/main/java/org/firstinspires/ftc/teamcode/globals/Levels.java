package org.firstinspires.ftc.teamcode.globals;

import java.util.HashMap;
import java.util.Map;

public enum Levels {
    INSTANCE;

    public enum TSELocation {
        LEVEL_3,
        LEVEL_2,
        LEVEL_1,
        NONE
    }

    Map<TSELocation, Integer> levels = new HashMap<>();


    Levels(){
        levels = new HashMap<>();
        levels.put(TSELocation.NONE,0);

        levels.put(TSELocation.LEVEL_1,1);
        levels.put(TSELocation.LEVEL_2,2);
        levels.put(TSELocation.LEVEL_3,3);

        setTSELocation(TSELocation.LEVEL_3);
    }
    private TSELocation selectedTSELocation = TSELocation.LEVEL_3;

    public void setTSELocation(TSELocation location){
        selectedTSELocation = location;
    }

    public TSELocation getTSELocation(){
        return selectedTSELocation;
    }
    public int getTSELevel(){
        //telemetry.addData("getTSELevel", location);
        return levels.get(getTSELocation());
    }

    public static Levels getInstance(){
        return INSTANCE;
    }
}
