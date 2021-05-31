package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class gm0_mecanum_field_centric extends OpMode {
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;
    BNO055IMU imu;
    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("BL");
        fr = hardwareMap.dcMotor.get("FR");
        br = hardwareMap.dcMotor.get("BR");
        // Reverse the right side motors
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        telemetry.addData("program", "initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x * 1.1; // counteract slower strafing
        double y = -gamepad1.left_stick_y; // reverse stick y axis
        double rx = gamepad1.right_stick_x;
        double heading = -imu.getAngularOrientation().firstAngle;

        x = (x*Math.cos(heading))-(y*Math.sin(heading));
        y = (x*Math.sin(heading))+(y*Math.cos(heading));

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
