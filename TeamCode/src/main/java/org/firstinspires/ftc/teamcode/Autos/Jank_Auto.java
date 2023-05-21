package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Jank_Auto extends LinearOpMode {
    private DcMotorEx motorBackLeft;
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorFrontLeft;

    double frontLeftPower = 0.75;
    double backLeftPower = 0.75;
    double frontRightPower = 0.75;
    double backRightPower = 0.75;

    @Override
    public void runOpMode(){


        // Expansion Hub Pins
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL"); // Pin 2
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("BL"); // Pin 1

        // Control Hub Pins
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR"); // Pin 3
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("BR"); // Pin 2

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Running without an encoder allows us t
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // to the motors total power. Ex. motor.s
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        int time = 1000;
        while(timer.milliseconds() <= time) {
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
    }
}
