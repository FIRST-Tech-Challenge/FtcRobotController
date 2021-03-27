package com.technototes.library.hardware.servo;

import com.technototes.library.hardware.HardwareDeviceGroup;
import com.technototes.logger.Log;

/** Class for servo group
 * @author Alex Stedman
 */
public class ServoGroup extends Servo implements HardwareDeviceGroup<Servo> {
    private Servo[] followers;

    /** Create a servo group
     *
     * @param leader The leader servo
     * @param followers The follower servos
     */
    public ServoGroup(Servo leader, Servo... followers) {
        super(leader.getDevice());
        this.followers = followers;
    }

    @Override
    public Servo[] getFollowers() {
        return followers;
    }

    @Override
    public Servo[] getAllDevices() {
        Servo[] m = new Servo[followers.length + 1];
        m[0] = this;
        System.arraycopy(followers, 0, m, 1, m.length - 1);
        return m;
    }

    @Override
    public void propogate(double value) {
        for(Servo s : followers){
            s.setPosition(value);
        }
    }

    @Override
    public void setPosition(double position) {
        super.setPosition(position);
        propogate(position);
    }
}