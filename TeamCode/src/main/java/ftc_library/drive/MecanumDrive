package ftc_library.drive;

import ftc_library.hardware.RobotHardware;

/**
 * MecanumDrive provides standard robot-centric drive methods for a mecanum wheelbase.
 * Use this class for basic movement in TeleOp or Autonomous.
 */
public class MecanumDrive {
    private final RobotHardware hw;

    public MecanumDrive(RobotHardware hardware) {
        this.hw = hardware;
    }

    /**
     * General drive method for mecanum wheels.
     * @param forward Forward/backward power
     * @param strafe  Left/right power
     * @param rotate  Rotation power
     */
    public void drive(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double bl = forward - strafe + rotate;
        double fr = forward - strafe - rotate;
        double br = forward + strafe - rotate;
        hw.frontLeft.setPower(fl);
        hw.backLeft.setPower(bl);
        hw.frontRight.setPower(fr);
        hw.backRight.setPower(br);
    }

    public void stop() { drive(0, 0, 0); }
    public void driveForward(double power) { drive(power, 0, 0); }
    public void driveBackward(double power) { drive(-power, 0, 0); }
    public void strafeLeft(double power) { drive(0, -power, 0); }
    public void strafeRight(double power) { drive(0, power, 0); }
    public void rotateLeft(double power) { drive(0, 0, -power); }
    public void rotateRight(double power) { drive(0, 0, power); }
}
