package org.firstinspires.ftc.forteaching.BasicOpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.forteaching.SimpleSwerveDrive;

@Disabled
@TeleOp(name = "Simple Swerve", group = "demo")
public class SimpleSwerve extends LinearOpMode {
    private static double ROT_DEAD_ZONE = 0.1;
    private static double POS_DEAD_ZONE = 0.1;
    private static double INV_ROOT_2 = 1.0 / Math.sqrt(2.0);
    SimpleSwerveDrive drive;
    BNO055IMU imu;

    private void hwinit() {
        DcMotorEx flm = hardwareMap.get(DcMotorEx.class, "flm");
        DcMotorEx frm = hardwareMap.get(DcMotorEx.class, "frm");
        DcMotorEx rlm = hardwareMap.get(DcMotorEx.class, "rlm");
        DcMotorEx rrm = hardwareMap.get(DcMotorEx.class, "rrm");
        Servo fls = hardwareMap.get(Servo.class, "fls");
        Servo frs = hardwareMap.get(Servo.class, "frs");
        Servo rls = hardwareMap.get(Servo.class, "rls");
        Servo rrs = hardwareMap.get(Servo.class, "rrs");
        drive = new SimpleSwerveDrive(flm, frm, rlm, rrm, fls, frs, rls, rrs);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    // Force a value to be between low & high (by adding/subtracting (high - low))
    // This is for degrees/radians conversions
    private static double inRange(double low, double high, double val) {
        // Dumb & slow, but easy :D
        while (val < low) {
            val += (high - low);
        }
        while (val >= high) {
            val -= (high - low);
        }
        return val;
    }

    // "clamp" a value to be no less than low, and no greater than high
    private static double clamp(double low, double high, double val) {
        if (val < low) return low;
        if (val > high) return high;
        return val;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hwinit();
        // This waits until the user hits the 'start' button :)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double rx = gamepad1.right_stick_x;
            double ry = gamepad1.right_stick_y;
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            if (Math.abs(rx) >= ROT_DEAD_ZONE) {
                // This lets you rotate a little slower by default
                // If you want to rotate faster, crank the y axis (either direction)
                double magnitude = Math.sqrt(rx * rx + ry * ry) * INV_ROOT_2;
                double sign = (rx < 0) ? -1.0 : 1.0;
                // Square the value to scale non-linearly
                drive.setRotationSpeed(magnitude * magnitude * sign);
            } else if (x >= POS_DEAD_ZONE || y >= POS_DEAD_ZONE) {
                // I have no idea if the angle to take the IMU position into account
                // for a driver-relative position is correct. The math should be *easy* but
                // it really does need to be *correct* :)
                float pos = imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle;
                double rad = inRange(-Math.PI, Math.PI, Math.atan2(y, x) - pos);
                double pow = clamp(-1, 1, Math.sqrt(y * y + x * x));
                drive.setDirectionAndPowerRadians(pow, rad);
            } else {
                drive.stop();
            }
        }
    }
}
