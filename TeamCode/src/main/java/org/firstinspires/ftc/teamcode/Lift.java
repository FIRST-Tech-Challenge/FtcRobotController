package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Lift {
    DcMotorEx lift;
    public static int LIFT_TOP = -1250;
    public static int LIFT_BOTTOM = 0;

    public Lift(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void drive(double input) {
        if (input < 0) {
            if (lift.getCurrentPosition() <= LIFT_TOP) {
                lift.setPower(0);
            } else {
                lift.setPower(input);
            }

        } else if (input >= 0) {
            if (lift.getCurrentPosition() >= LIFT_BOTTOM) {
                lift.setPower(0);
            } else {
                lift.setPower(input);
            }
        }

    }

    public double encoderValue() {
        return lift.getCurrentPosition();
    }
}
