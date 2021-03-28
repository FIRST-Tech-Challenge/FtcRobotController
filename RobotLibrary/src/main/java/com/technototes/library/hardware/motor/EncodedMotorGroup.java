package com.technototes.library.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.technototes.library.hardware.HardwareDeviceGroup;
import com.technototes.logger.Log;

/** Class for encoded motor groups
 * @author Alex Stedman
 */
public class EncodedMotorGroup extends EncodedMotor<DcMotorSimple> implements HardwareDeviceGroup<Motor<?>> {
    private Motor<?>[] followers;

    /** Create an encoded motor group
     *
     * @param leader The Lead motor
     * @param followers The following motors
     */
    public EncodedMotorGroup(EncodedMotor<?> leader, Motor<?>... followers) {
        super(leader.getDevice());
        this.followers = followers;

    }

    @Override
    public Motor<?>[] getFollowers() {
        return followers;
    }

    @Override
    public Motor<?>[] getAllDevices() {
        Motor<?>[] m = new Motor[followers.length + 1];
        m[0] = this;
        System.arraycopy(followers, 0, m, 1, m.length - 1);
        return m;
    }

    @Override
    public void propogate(double value) {
        for(Motor m : followers){
            m.setSpeed(value);
        }
    }


    @Override
    public void setVelocity(double tps) {
        super.setVelocity(tps);
        propogate(super.getSpeed());
    }

    @Override
    public boolean setPosition(double ticks, double speed) {
        boolean b = super.setPosition(ticks, speed);
        propogate(super.getSpeed());
        return b;
    }
}
