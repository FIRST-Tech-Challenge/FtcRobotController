package org.firstinspires.ftc.teamcode.robot.parts.drivetrain.wheels;

/**
 * Ways wheels can fail
 * @author 22jmiller
 */
public class WheelException extends RuntimeException {
    public WheelException() {
        super("This wheel is not supported on this drivetrain");
    }

    public WheelException(String reason) {
        super(reason);
    }
}
