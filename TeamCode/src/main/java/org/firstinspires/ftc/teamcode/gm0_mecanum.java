package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class gm0_mecanum extends OpMode {
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;
    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");

        // Reverse the right side motors
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // reverse stick y axis
        double x = gamepad1.left_stick_x * 1.1; // counteract slower strafing
        double rx = gamepad1.right_stick_x;

        double flPower = y + x + rx;
        double blPower = y - x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;

        // put powers in the range of -1 to 1 only if they aren't already
        // this corrects the power ratio between all the wheels and makes sure
        // the robot goes in the right direction
        if (Math.abs(flPower) > 1 || Math.abs(blPower) > 1 ||
                Math.abs(frPower) > 1 || Math.abs(brPower) > 1) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(flPower), Math.abs(blPower));
            max = Math.max(Math.abs(frPower), max);
            max = Math.max(Math.abs(brPower), max);

            // Divide everything by max
            flPower /= max;
            blPower /= max;
            frPower /= max;
            brPower /= max;
        }

        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);
    }
}
