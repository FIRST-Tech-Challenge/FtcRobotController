package org.firstinspires.ftc.teamcode.globals;

public enum Side {
    INSTANCE;

    public enum PositionSide {
        DUCKSIDE,
        WAREHOUSESIDE
    }

    private PositionSide selectedPositionSide = PositionSide.DUCKSIDE;

    public void setPositionSide(PositionSide positionSide){
        selectedPositionSide = positionSide;
    }

    public PositionSide getPositionSide(){
        return selectedPositionSide;
    }

    public static Side getInstance(){
        return INSTANCE;
    }
}
