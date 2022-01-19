package org.firstinspires.ftc.teamcode.globals;

public enum Levels {
    INSTANCE;

    public enum TSELocation {
        LEVEL_3,
        LEVEL_2,
        LEVEL_1,
        NONE
    }

    private TSELocation selectedTSELocation = TSELocation.LEVEL_3;

    public void setTSELocation(TSELocation location){
        selectedTSELocation = location;
    }

    public TSELocation getTSELocation(){
        return selectedTSELocation;
    }

    public static Levels getInstance(){
        return INSTANCE;
    }
}
