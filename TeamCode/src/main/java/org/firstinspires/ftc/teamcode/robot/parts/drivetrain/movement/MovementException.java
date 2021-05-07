package org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement;

/**
 * Failure to move
 * @author 22jmiller;
 */
public class MovementException extends RuntimeException {
    private MovementExceptionReason reason;

    public MovementException(MovementExceptionReason reason) {
        super(reason.name());
        this.reason = reason;
    }
}
