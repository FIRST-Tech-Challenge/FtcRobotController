package org.firstinspires.ftc.teamcode.teleop.newTeleop.main;
import org.firstinspires.ftc.teamcode.mainEnum;


public interface teleop_interface {
    void initialize();
    void telemetryInit();
    void setDirection();
    void setBrakes();

    void whileMotorsBusy();

    void movement(double vertical, double strafe, double turn);
    void arm(mainEnum state, double speed);
    void claw(mainEnum state, double power, int pos);

    void finalMovement();
    void finalArm();
    void finalGrabber();
}
