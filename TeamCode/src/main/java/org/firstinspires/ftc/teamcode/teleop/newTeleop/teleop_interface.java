package org.firstinspires.ftc.teamcode.teleop.newTeleop;


public interface teleop_interface {
    void initialize();
    void telemetryInit();
    void setDirection();

    void whileMotorsBusy();

    void movement(double vertical, double strafe, double turn);
    void arm(teleop_enum state, double speed);
    void claw(teleop_enum state, int pos);

    void finalMovement();
    void finalArm();
    void finalGrabber();
}
