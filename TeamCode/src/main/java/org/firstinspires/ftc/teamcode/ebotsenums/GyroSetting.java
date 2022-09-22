package org.firstinspires.ftc.teamcode.ebotsenums;

public enum GyroSetting {
    NONE(false, -1),
    INFREQUENT (true, 10),
    EVERY_LOOP (true, 1);

    /**  ENUM VARIABLES     **************/
    private boolean gyroOn;
    private int readFrequency;

    /**  CONSTRUCTOR    **************/
    GyroSetting (boolean gyroPower, int frequency){
        this.gyroOn = gyroPower;
        this.readFrequency = frequency;
    }

    /**  ENUM GETTERS AND SETTERS  ***********/
    public Boolean isGyroOn(){return this.gyroOn;}
    public Integer getReadFrequency(){return this.readFrequency;}

}
