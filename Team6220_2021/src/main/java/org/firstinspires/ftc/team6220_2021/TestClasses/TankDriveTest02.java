//  This is a class for TeleOp tank drive.
//  You can install this program to the rev control hub without any edits.
//  Using the logitech controller, you can move the robot in tank drive.

package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TankDriveTest02", group = "TeleOp")
public class TankDriveTest02 extends LinearOpMode {

    //Motors
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorDuck;


    //Other Devices
    BNO055IMU imu;

    @Override
    public void runOpMode() {

        //Initialize
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorDuck = hardwareMap.dcMotor.get("motorDuck");

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Init", "Done");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            motorBackLeft.setPower((gamepad1.left_stick_y));
            motorFrontLeft.setPower((gamepad1.left_stick_y));

            motorBackRight.setPower((gamepad1.right_stick_y));
            motorFrontRight.setPower((gamepad1.right_stick_y));

            //Telemetry
            telemetry.addData("MotorBackLeftPower: ", -motorBackLeft.getPower());
            telemetry.addData("MotorBackRightPower: ", -motorBackRight.getPower());
            telemetry.addData("motorFrontLeft: ", -motorFrontLeft.getPower());
            telemetry.addData("motorFrontRight: ", -motorFrontRight.getPower());

            telemetry.update();

        }
    }
}