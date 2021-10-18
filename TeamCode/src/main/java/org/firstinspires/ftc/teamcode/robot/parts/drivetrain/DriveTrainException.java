package org.firstinspires.ftc.teamcode.robot.parts.drivetrain;

/**
 * Ways the Drive train can fail
 * @author 22jmiller
 */
public class DriveTrainException extends RuntimeException {
    public DriveTrainException() {
        super("Drive Train Exception");
    }
    public DriveTrainException(String msg) {
        super(msg);
    }
}
