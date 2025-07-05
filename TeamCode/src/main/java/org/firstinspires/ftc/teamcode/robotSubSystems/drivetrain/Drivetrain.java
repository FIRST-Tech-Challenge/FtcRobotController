package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.Vector;

public class Drivetrain {
    private static DcMotor lf;
    private static DcMotor rf;
    private static DcMotor lb;
    private static DcMotor rb;


    public static void init(HardwareMap hardwareMap) {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        Gyro.init(hardwareMap);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    public static void operate(final Vector joystick, final float omega) {
        final float robotAngle = (float) Math.toRadians(Angle.wrapPlusMinusPI(Gyro.getAngle()));
        final Vector velocity_RobotCS_W = joystick.rotate(-robotAngle);

        if (velocity_RobotCS_W.norm() < 0.01 && Math.abs(omega) == 0) {
            stop();
        } else {
            drive(joystick, omega);
        }
    }

    public static void drive(Vector drive, final float r) {


        final double lfPower = drive.y + drive.x + r ;
        final double rfPower = drive.y - drive.x - r;
        final double lbPower = drive.y - drive.x + r;
        final double rbPower = drive.y + drive.x - r;


        double highestPower = 1;
        final double max = Math.max(Math.abs(lfPower),
                Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower))));

        if (max > 1) highestPower = max;

        lf.setPower(lfPower / highestPower);
        rf.setPower(rfPower / highestPower);
        lb.setPower(lbPower / highestPower);
        rb.setPower(rbPower / highestPower);

    }

    public static void stop() {
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
    }

    public static void testMotors(Gamepad gamepad) {
        if (gamepad.dpad_left) {
            lf.setPower(0.2);
        } else if (gamepad.dpad_right) {
            rf.setPower(0.2);
        } else if (gamepad.dpad_down) {
            lb.setPower(0.2);
        } else if (gamepad.dpad_up) {
            rb.setPower(0.2);
        }
    }
}
