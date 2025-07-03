package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    public static void operate(final Vector joystick,final float omega){
        final Vector fieldCSJoystick = joystick.rotate(Angle.wrapPlusMinusPI((float) -Math.toRadians(Gyro.getAngle())));

        if (fieldCSJoystick.norm() < 0.01 && Math.abs(omega) == 0){
            stop();
        }else {
            drive(fieldCSJoystick.y,fieldCSJoystick.x,omega);
        }
    }

    public static void drive(final float y, final float x, final float omega) {

        lf.setPower(y + x + omega);
        rf.setPower(y - x - omega);
        lb.setPower(y - x + omega);
        rb.setPower(y + x - omega);
    }

    public static void stop() {
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
    }
}