package com.technototes.subsystem;

public interface DrivebaseSubsystem extends SpeedSubsystem{
    enum DriveSpeed{
        SNAIL(0.2), NORMAL(0.5), TURBO(1);
        public double spe;
        DriveSpeed(double s){
            spe = s;
        }
        public double getSpeed(){
            return spe;
        }
    }
    default double getScale(double... powers){
        double max = 0;
        for(double d : powers){
            max = Math.abs(d) > max ? Math.abs(d) : max;
        }
        return max;
    }
    void setDriveSpeed(DriveSpeed ds);
    DriveSpeed getDriveSpeed();

    @Override
    default double getSpeed(){
        return getDriveSpeed().getSpeed();
    }

    @Override
    default void setSpeed(double speed){

    }
}
