package org.firstinspires.ftc.teamcode.robotSubSystems.Elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    private static DcMotor rightMotor;
    private static DcMotor leftMotor;
    public static void init(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotor.class, "1");
        leftMotor = hardwareMap.get(DcMotor.class, "0");
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public static void operate(double val) {
        rightMotor.setPower(val);
        leftMotor.setPower(val);
    }

}
