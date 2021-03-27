package com.technototes.subsystem;

/** An interface for drivebase subsystems
 * @author Alex Stedman
 */
public interface DrivebaseSubsystem extends SpeedSubsystem {
    /** Enum for built in drive speeds (Deprecated because its better to do it custom)
     *
     */
    @Deprecated
    enum SampleDriveSpeed {
        SNAIL(0.2), NORMAL(0.5), TURBO(1);
        public double spe;
        SampleDriveSpeed(double s){
            spe = s;
        }
        public double getSpeed(){
            return spe;
        }
    }

    /** Returns the maximum of the given doubles, for better speed scaling
     *
     * @param powers Motor powers
     * @return The maximum of all the given powers
     */
    default double getScale(double... powers){
        double max = 0;
        for(double d : powers){
            max = Math.abs(d) > max ? Math.abs(d) : max;
        }
        return max;
    }

    /** Returns the Drivespeed
     *
     * @return The Drivespeed
     */
    @Deprecated
    default SampleDriveSpeed getSampleDriveSpeed(){
        return SampleDriveSpeed.NORMAL;
    }

    /** Return DriveSpeed as a double (again, dont use built in speeds, override this method)
     *
     * @return The double
     */
    @Override
    default double getSpeed(){
        return getSampleDriveSpeed().getSpeed();
    }
    @Override
    default void setSpeed(double speed){

    }
}
