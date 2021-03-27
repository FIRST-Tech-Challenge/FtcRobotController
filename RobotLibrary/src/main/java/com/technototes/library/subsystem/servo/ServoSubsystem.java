package com.technototes.library.subsystem.servo;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.Subsystem;

/** Class for servo subsystems
 * @author Alex Stedman
 */
public class ServoSubsystem extends Subsystem<Servo> {
    /** Create servo subsystem
     *
     * @param servos The servos
     */
    public ServoSubsystem(Servo... servos) {
        super(servos);
    }

    /** Set servo subsystem position
     *
     * @param position The position
     */
    public void setPosition(double position) {
        for (Servo m : getDevices()) {
            m.setPosition(position);
        }
    }

    /** Get subsystem servo position
     *
     * @return The position
     */
    public double getPosition(){
        return getDevices()[0].getPosition();
    }

}
