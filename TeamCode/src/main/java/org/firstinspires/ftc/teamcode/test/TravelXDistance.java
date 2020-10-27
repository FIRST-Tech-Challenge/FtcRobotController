package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Move 10 centimeters", group="Auton Test Suite")
public class TravelXDistance extends LinearOpMode
{
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    double leftFrontMotorPos, rightBackMotorPos;
    double deltaLeftFrontMotorPos = 0;
    double previousLeftFrontMotorPos = 0;
    double leftFrontDistanceTraveled = 0;
    double deltaRightBackMotorPos = 0;
    double previousRightBackMotorPos = 0;
    double rightBackDistanceTraveled = 0;
    double distancePerTick = (2 * Math.PI * 48) / 537.6;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("1 ticks per cycle", leftFrontMotorPos);
            telemetry.addData("2 distance traveled per cycle", deltaLeftFrontMotorPos);
            telemetry.addData("3 total distance traveled", leftFrontDistanceTraveled);
            telemetry.update();
            move(100);
        }
    }

    void move(double target) {
        leftFrontMotorPos = leftFrontMotor.getCurrentPosition();
        deltaLeftFrontMotorPos = distancePerTick * (leftFrontMotorPos - previousLeftFrontMotorPos);
        leftFrontDistanceTraveled += deltaLeftFrontMotorPos;
        previousLeftFrontMotorPos = leftFrontMotorPos;

        rightBackMotorPos = rightBackMotor.getCurrentPosition();
        deltaRightBackMotorPos = distancePerTick * (rightBackMotorPos - previousRightBackMotorPos);
        rightBackDistanceTraveled += deltaRightBackMotorPos;
        previousRightBackMotorPos = rightBackMotorPos;

        if (leftFrontDistanceTraveled < target) {
            leftFrontMotor.setPower(0.2);
            rightBackMotor.setPower(-0.2);
        } else if (leftFrontDistanceTraveled >= target) {
            leftFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
    }
}
