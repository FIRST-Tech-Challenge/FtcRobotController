package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.settings.DeviceNames;

import java.util.ArrayList;

@Disabled
@Deprecated
@TeleOp (name="Josh: 4 Wheel AWD Test", group="Josh")
public class Drive extends OpMode {
    ArrayList<DcMotor> leftDriveMotors;
    ArrayList<DcMotor>  rightDriveMotors;

    @Override
    public void init() {
        // Left Drive
        leftDriveMotors = new ArrayList<DcMotor>();
        for (String name : DeviceNames.LEFT_DRIVE) {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftDriveMotors.add(motor);
        }

        // Right Drive
        rightDriveMotors = new ArrayList<DcMotor>();
        for (String name : DeviceNames.RIGHT_DRIVE) {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            rightDriveMotors.add(motor);
        }
    }

    // Left is -
    // Up is +
    public double[] getPowerOneWay(Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double totalPower = Math.sqrt(Math.pow(x,2) + Math.pow(y,2)); // Distance Formula

        double lPower = 0;
        double rPower = 0;
        if (y >= 0) { // Going Forwards
            // y = mx + b
            rPower = x + totalPower;
            lPower = -x + totalPower;

            // Cap to 1
            lPower = Math.min(lPower,1);
            rPower = Math.min(rPower,1);
        } else {
            totalPower *= -1;
            // y = mx + b
            rPower = x + totalPower;
            lPower = -x + totalPower;

            // Cap to -1
            //lPower = Math.max(lPower,-1);
            //rPower = Math.max(rPower,-1);
        }


        System.out.println(totalPower);
        //double lPower = gamepad.left_stick_y;
        //double rPower = gamepad.right_stick_y;
        return new double[] {lPower, rPower};
    }

    public double[] getPowerTank(Gamepad gamepad) {
        double lPower = gamepad.left_stick_y;
        double rPower = gamepad.right_stick_y;
        return new double[] {lPower, rPower};
    }



    @Override
    public void loop() {
        double[] array = getPowerOneWay(gamepad1);
        double lPower = array[0];
        double rPower = array[1];
        for (DcMotor leftMotor : leftDriveMotors) {
            leftMotor.setPower(lPower);
        }

        for (DcMotor rightMotor : rightDriveMotors) {
            rightMotor.setPower(rPower);
        }

        telemetry.addData("Motors ", lPower + "l | " + rPower + "r");
        telemetry.addData("Controller", gamepad1.left_stick_x + "x | " + gamepad1.left_stick_y);
    }
}
