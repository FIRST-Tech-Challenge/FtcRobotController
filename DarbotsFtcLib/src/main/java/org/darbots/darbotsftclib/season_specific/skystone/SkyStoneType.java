package org.darbots.darbotsftclib.season_specific.skystone;

public enum SkyStoneType {
    UNKNOWN(0),
    SKYSTONE(1),
    STONE(2);

    private int value;
    private SkyStoneType(int val){
        value = val;
    }
    public int value(){
        return this.value;
    }
    public static SkyStoneType valueOf(int value){
        switch(value){
            case 1:
                return SKYSTONE;
            case 2:
                return STONE;
            default:
                return UNKNOWN;
        }
    }
}
