package com.kalipsorobotics.PID;

import android.annotation.SuppressLint;
import android.os.SystemClock;

import com.kalipsorobotics.localization.OdometrySpark;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Position;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class TestingDriveTrain {
    private final DcMotor fLeft, fRight, bLeft, bRight;
    public final SparkFunOTOS otos;
    public final PIDController xController;
    public final PIDController yController;
    public final PIDController headingController;
    public final OdometrySpark odometryFuse;

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
        odometryFuse = new OdometrySpark(otos);
        odometryFuse.configureOtos(otos);

        // tuned w/ lightweight testing drivetrain
        xController = new PIDController(0.075, 0.015, 0.01, "xController");
        yController = new PIDController(0.05, 0.0065, 0.012, "yController");
        headingController = new PIDController(0.01, 0.0005, 0, "hController");
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
        double startingTime = SystemClock.elapsedRealtimeNanos();
        Position pos = new Position(odometryFuse.sparkUpdateData().getX(), odometryFuse.sparkUpdateData().getY(), odometryFuse.headingUpdateData("right", 0, 0));

        double curX = -pos.getX();
        double curY = -pos.getY();
        double curH = MathFunctions.angleWrapDeg(odometryFuse.headingUpdateData("left", 0, 0));

        Pose2D target = new Pose2D(curX + dx, curY + dy, curH + dh);
        System.out.println(xController);
        System.out.println(yController);
        System.out.println(headingController);

        while (
                (Math.abs(target.x - curX) > 0.1 || Math.abs(target.y - curY) > 0.1 || Math.abs(target.h - curH) > 2)
                        && ((SystemClock.elapsedRealtimeNanos() - startingTime) / 1e9) < 5
        ) {  // I think this while loop is causing an exit error
            pos = new Position(odometryFuse.sparkUpdateData().getX(), odometryFuse.sparkUpdateData().getY(), odometryFuse.headingUpdateData("right", 0, 0));
            curX = -pos.getX();  // odometryfuse returns negative
            curY = -pos.getY();
            curH = MathFunctions.angleWrapDeg(odometryFuse.headingUpdateData("left", 0, 0));

            double x = Range.clip(xController.calculate(curX, target.x), -1., 1.);
            double y = Range.clip(yController.calculate(curY, target.y), -1., 1.);
            double h = Range.clip(headingController.calculate(curH, target.h), -1., 1.);

            setPowers(y + x + h, y - x - h, y - x + h, y + x - h);

            telemetry.addLine(String.format("x | currently at %f, targeting %f, power %f\n", curX, target.x, x));
            telemetry.addLine(String.format("y | currently at %f, targeting %f, power %f\n", curY, target.y, y));
            telemetry.addLine(String.format("h | currently at %f, targeting %f, power %f\n", curH, target.h, h));
            telemetry.update();

            System.out.printf("x | currently at %f, targeting %f, power %f\n", curX, target.x, x);
            System.out.printf("y | currently at %f, targeting %f, power %f\n", curY, target.y, y);
            System.out.printf("h | currently at %f, targeting %f, power %f\n", curH, target.h, h);
        }
        setPowers(0, 0, 0, 0);
    }
}
