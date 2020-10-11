package org.darbots.darbotsftclib.libcore.templates.servo_related.motor_powered_servos;

public interface RobotServoUsingMotorCallBack {
    void JobFinished(boolean timeOut, RobotServoUsingMotorTask Task, double OriginalPos, double timeSpent);
}
