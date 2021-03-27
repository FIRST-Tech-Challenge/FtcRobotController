package com.technototes.subsystem;

import com.qualcomm.robotcore.util.Range;

/** An interface for non holomnic drivebases(Tank for example)
 * @author Alex Stedman
 */
public interface NonHolomonicDrivebaseSubsystem extends DrivebaseSubsystem {
    /** Basic robot joystick drive function
     *
     * @param y The stick y value
     * @param x The stick x value
     */
    default void arcadeDrive(double y, double x){
        double lp = y+x;
        double rp = -y+x;
        double scale = getScale(lp, rp);
        double speed = Range.clip(Math.abs(y)+Math.abs(x), 0, 1);
        scale = scale == 0 ? 0 : speed/scale;
        drive(lp*scale, rp*scale);
    }

    /** Basic drive function
     *
     * @param lpower The power to the left motor side
     * @param rpower The power to the right motor side
     */
    void drive(double lpower, double rpower);

    /** Stop the drivebase
     *
     */
    @Override
    default void stop() {
        drive(0, 0);
    }
}
