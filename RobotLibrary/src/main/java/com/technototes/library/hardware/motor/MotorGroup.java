package com.technototes.library.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.technototes.library.hardware.HardwareDeviceGroup;
import com.technototes.library.hardware.servo.Servo;

/** Class for a group of motors
 *
 * @param <T> The type of motors to group
 */
public class MotorGroup<T extends Motor> extends Motor<DcMotorSimple> implements HardwareDeviceGroup<Motor> {
    private Motor[] followers;

    /** Make a motor group
     *
     * @param leader The leader motor
     * @param followers The follower motors
     */
    public MotorGroup(Motor<DcMotorSimple> leader, Motor... followers) {
        super(leader.getDevice());
        this.followers = followers;
    }

    @Override
    public Motor[] getFollowers() {
        return followers;
    }

    @Override
    public Motor[] getAllDevices() {
        Motor[] m = new Motor[followers.length + 1];
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
    public void setSpeed(double speed) {
        super.setSpeed(speed);
        propogate(speed);
    }
}
