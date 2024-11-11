package com.kalipsorobotics.PID;

import android.annotation.SuppressLint;

import com.kalipsorobotics.localization.OdometryFuse;
import com.kalipsorobotics.math.Point;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class TestingDriveTrain {
    private final DcMotor fLeft, fRight, bLeft, bRight;
    public final SparkFunOTOS otos;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;
    public final OdometryFuse odometryFuse;


    public TestingDriveTrain(HardwareMap hardwareMap) {
        fLeft = hardwareMap.get(DcMotor.class, "fLeft");
        fRight = hardwareMap.get(DcMotor.class, "fRight");
        bLeft = hardwareMap.get(DcMotor.class, "bLeft");
        bRight = hardwareMap.get(DcMotor.class, "bRight");

        fLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);

        DcMotor[] motors = {fLeft, fRight, bLeft, bRight};
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        otos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
        odometryFuse = new OdometryFuse(otos, fRight, bRight);
        odometryFuse.configureOtos(otos);

        xController = new PIDController(0.075, 0.015, 0.01);  // placeholder values
        yController = new PIDController(0.05, 0.0065, 0.012);
        headingController = new PIDController(0, 0, 0);
    }

    public void setPowers(double fLeftP, double fRightP, double bLeftP, double bRightP) {
        double max = Math.max(
            Math.max(Math.abs(fLeftP), Math.abs(fRightP)),
            Math.max(Math.abs(bLeftP), Math.abs(bRightP))
        );
        fLeft.setPower(fLeftP / Math.max(1., max));
        fRight.setPower(fRightP / Math.max(1., max));
        bLeft.setPower(bLeftP / Math.max(1., max));
        bRight.setPower(bRightP / Math.max(1., max));
    }

    @SuppressLint("DefaultLocale")
    public void move(double dx, double dy, double dh, Telemetry telemetry) {
        Point pos = odometryFuse.PointCollectData();  // current
        double curX = pos.getX();
        double curY = pos.getY();
        double curH = otos.getPosition().h;

        Pose2D target = new Pose2D(curX + dx, curY + dy, curH + dh);

        while (Math.abs(target.x - curX) > 0.2 ||
                Math.abs(target.y - curY) > 0.2 ||
                Math.abs(target.h - curH) > 5
        ) {  // I think this while loop is causing an exit error
            pos = odometryFuse.PointCollectData();
            curX = pos.getX();
            curY = pos.getY();
            curH = otos.getPosition().h;

            double x = Range.clip(xController.calculate(curX, target.x), -1., 1.);
            double y = Range.clip(yController.calculate(curY, target.y), -1., 1.);
            double h = Range.clip(headingController.calculate(curH, target.h), -1., 1.);

            setPowers(y + x + h, y - x - h, y - x + h, y + x - h);

            telemetry.addLine(String.format("x | currently at %f, targeting %f, power %f\n", curX, target.x, x));
            telemetry.addLine(String.format("y | currently at %f, targeting %f, power %f\n", curY, target.y, y));
            telemetry.addLine(String.format("h | currently at %f, targeting %f, power %f\n", curH, target.h, h));

            System.out.printf("x | currently at %f, targeting %f, power %f\n", curX, target.x, x);
            System.out.printf("y | currently at %f, targeting %f, power %f\n", curY, target.y, y);
//            System.out.printf("h | currently at %f, targeting %f, power %f\n", curH, target.h, h);
        }
        setPowers(0, 0, 0, 0);
    }
}
