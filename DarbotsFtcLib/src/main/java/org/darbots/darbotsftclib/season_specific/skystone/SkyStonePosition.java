package org.darbots.darbotsftclib.season_specific.skystone;

public enum SkyStonePosition {
    UNKNOWN(0),
    NEXT_TO_BRIDGE(1),
    MIDDLE(2),
    NEXT_TO_WALL(3);

    private int value = 0;
    private SkyStonePosition(int val){
        this.value = val;
    }
    public int value(){
        return this.value;
    }
    public static SkyStonePosition valueOf(int value){
        switch(value){
            case 1:
                return NEXT_TO_BRIDGE;
            case 2:
                return MIDDLE;
            case 3:
                return NEXT_TO_WALL;
            default:
                return UNKNOWN;
        }
    }
}
