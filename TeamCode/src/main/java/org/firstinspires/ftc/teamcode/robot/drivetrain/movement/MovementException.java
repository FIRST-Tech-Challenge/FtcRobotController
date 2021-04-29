package org.firstinspires.ftc.teamcode.robot.drivetrain.movement;

public class MovementException extends RuntimeException {
    private MovementExceptionReason reason;

    public MovementException(MovementExceptionReason reason) {
        super(reason.name());
        this.reason = reason;
    }
}
