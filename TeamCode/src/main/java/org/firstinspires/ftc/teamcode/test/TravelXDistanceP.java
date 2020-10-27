package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Move 10 centimeters with proportional control", group="Auton Test Suite")
public class TravelXDistanceP extends LinearOpMode
{
    DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    double distancePerTick = (2 * Math.PI * 48) / 537.6;
    public double leftFrontMotorPos, leftFrontDistanceTraveled = 0, deltaLeftFrontMotorPos = 0, previousLeftFrontMotorPos;
    public double rightFrontMotorPos, rightFrontDistanceTraveled = 0, deltaRightFrontMotorPos = 0, previousRightFrontMotorPos;
    public double leftBackMotorPos, leftBackDistanceTraveled = 0, deltaLeftBackMotorPos = 0, previousLeftBackMotorPos;
    public double rightBackMotorPos, rightBackDistanceTraveled = 0, deltaRightBackMotorPos = 0, previousRightBackMotorPos;
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("1 LF distance traveled", leftFrontDistanceTraveled);
            telemetry.addData("2 RF distance traveled", rightFrontDistanceTraveled);
            telemetry.addData("3 LB distance traveled", leftBackDistanceTraveled);
            telemetry.addData("4 RB distance traveled", rightBackDistanceTraveled);
            telemetry.update();
            move(-300, 300);
        }
    }

    void move(double xTarget, double yTarget) {
        leftFrontMotorPos = leftFrontMotor.getCurrentPosition();
        deltaLeftFrontMotorPos = distancePerTick * (leftFrontMotorPos - previousLeftFrontMotorPos);
        leftFrontDistanceTraveled += deltaLeftFrontMotorPos;
        previousLeftFrontMotorPos = leftFrontMotorPos;

        rightFrontMotorPos = rightFrontMotor.getCurrentPosition();
        deltaRightFrontMotorPos = distancePerTick * (rightFrontMotorPos - previousRightFrontMotorPos);
        rightFrontDistanceTraveled += deltaRightFrontMotorPos;
        previousRightFrontMotorPos = rightFrontMotorPos;

        leftBackMotorPos = leftBackMotor.getCurrentPosition();
        deltaLeftBackMotorPos = distancePerTick * (leftBackMotorPos - previousLeftBackMotorPos);
        leftBackDistanceTraveled += deltaLeftBackMotorPos;
        previousLeftBackMotorPos = leftBackMotorPos;

        rightBackMotorPos = rightBackMotor.getCurrentPosition();
        deltaRightBackMotorPos = distancePerTick * (rightBackMotorPos - previousRightBackMotorPos);
        rightBackDistanceTraveled += deltaRightBackMotorPos;
        previousRightBackMotorPos = rightBackMotorPos;

        updateControlLoop(xTarget, yTarget);
    }

    double getAngle()
    {
        double angle = 0;
        if (rightFrontPower > 0) { angle = (Math.atan(-rightFrontPower / leftFrontPower) - Math.PI / 4); }
        if (rightFrontPower < 0) { angle = (Math.atan(-rightFrontPower / leftFrontPower) + Math.PI - Math.PI / 4); }
        return angle;
    }

    void updateControlLoop(double xTarget, double yTarget)
    {
        if (leftFrontDistanceTraveled < xTarget) { leftFrontPower = Math.pow(runtime.seconds(), 2); }
        if (rightFrontDistanceTraveled < yTarget) { rightFrontPower = Math.pow(runtime.seconds(), 2); }
        if (leftBackDistanceTraveled < xTarget) { leftBackPower = Math.pow(runtime.seconds(), 2); }
        if (rightBackDistanceTraveled < yTarget) { rightBackPower = Math.pow(runtime.seconds(), 2); }

        if (leftFrontPower > 0.35) { leftFrontPower = 0.35; }
        if (rightFrontPower > 0.35) { rightFrontPower = 0.35; }
        if (leftBackPower > 0.35) { leftBackPower = 0.35; }
        if (rightBackPower > 0.35) { rightBackPower = 0.35; }

        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }
}
