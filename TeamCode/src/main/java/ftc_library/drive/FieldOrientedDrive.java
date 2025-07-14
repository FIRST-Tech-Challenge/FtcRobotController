package ftc_library.drive;

import ftc_library.hardware.RobotHardware;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * FieldOrientedDrive enables field-centric driving using the IMU.
 * The robot moves relative to the field, not its own orientation.
 */
public class FieldOrientedDrive {
    private final RobotHardware hw;

    public FieldOrientedDrive(RobotHardware hardware) {
        this.hw = hardware;
    }

    /**
     * Field-oriented drive: robot moves in the direction the joystick is pushed, relative to the field.
     * @param gamepad      Gamepad input for driver
     * @param slowMode     If true, reduces speed for precision
     * @param rightBumper  If true, halves speed for fine control
     */
    public void drive(Gamepad gamepad, boolean slowMode, boolean rightBumper) {
        double y = -gamepad.right_stick_y;
        double x = gamepad.right_stick_x;
        double rx = gamepad.left_stick_x;

        y = RobotHardware.powerCurve(y, slowMode);
        x = RobotHardware.powerCurve(x, slowMode);
        rx = RobotHardware.powerCurve(rx, slowMode);

        // Get robot's current heading from the IMU
        YawPitchRollAngles angles = hw.imu.getRobotYawPitchRollAngles();
        double yaw = angles.getYaw(AngleUnit.RADIANS);

        // Transform joystick input to field coordinates
        double rotX = x * Math.cos(-yaw) - y * Math.sin(-yaw);
        double rotY = x * Math.sin(-yaw) + y * Math.cos(-yaw);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        if (rightBumper) denominator /= 2.0;

        double fl = (rotY + rotX + rx) / denominator;
        double bl = (rotY - rotX + rx) / denominator;
        double fr = (rotY - rotX - rx) / denominator;
        double br = (rotY + rotX - rx) / denominator;

        hw.frontLeft.setPower(fl);
        hw.backLeft.setPower(bl);
        hw.frontRight.setPower(fr);
        hw.backRight.setPower(br);
    }

    /**
     * Emergency drive: direct robot-centric control (ignores field orientation).
     * Useful if IMU fails or for debugging.
     */
    public void emergencyDrive(Gamepad gamepad, boolean rightBumper, boolean leftBumper) {
        double y = -gamepad.right_stick_y;
        double x = gamepad.right_stick_x;
        double rx = gamepad.left_stick_x;
        double dividedBy = rightBumper ? 0.5 : (leftBumper ? 4.0 : 2.0);

        hw.frontLeft.setPower((y + x + rx) / dividedBy);
        hw.backLeft.setPower((y - x + rx) / dividedBy);
        hw.frontRight.setPower((y - x - rx) / dividedBy);
        hw.backRight.setPower((y + x - rx) / dividedBy);
    }
}
