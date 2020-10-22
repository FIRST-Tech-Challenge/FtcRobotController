package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Move 5 revolutions", group="Auton Test Suite")
@Disabled
public class TravelXRevolutions extends LinearOpMode
{
    DcMotor leftFrontMotor;
    double leftFrontMotorPos;
    double deltaLeftFrontMotorPos = 0;
    double previousLeftFrontMotorPos = 0;
    double leftFrontRevolutions = 0;
    double ticksPerRevolution = 537.6;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("1 ticks per cycle", leftFrontMotorPos);
            telemetry.addData("2 distance traveled per cycle", deltaLeftFrontMotorPos);
            telemetry.addData("3 total revolutions traveled", leftFrontRevolutions);
            telemetry.update();

            move();
        }
    }

    void move()
    {
        leftFrontMotorPos = leftFrontMotor.getCurrentPosition();
        deltaLeftFrontMotorPos = (leftFrontMotorPos - previousLeftFrontMotorPos) / ticksPerRevolution;
        leftFrontRevolutions += deltaLeftFrontMotorPos;
        previousLeftFrontMotorPos = leftFrontMotorPos;

        if (leftFrontRevolutions < 5) {
            leftFrontMotor.setPower(0.2);
        } else if (leftFrontRevolutions >= 5) {
            leftFrontMotor.setPower(0);
        }
    }
}
