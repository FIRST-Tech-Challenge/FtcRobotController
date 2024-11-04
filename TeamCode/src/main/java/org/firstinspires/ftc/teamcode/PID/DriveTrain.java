package org.firstinspires.ftc.teamcode.PID;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class DriveTrain {
    private final DcMotor fLeft, fRight, bLeft, bRight;
    private final OTOS otos;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;


    public DriveTrain(HardwareMap hardwareMap) {
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

        otos = new OTOS(hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS"));
        otos.configureOtos();
        xController = new PIDController(0.04, 0.001, 0);  // placeholder values
        yController = new PIDController(0.04, 0.001, 0);
        headingController = new PIDController(0.05, 0, 0);
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
        Pose2D pos = otos.getPosition();  // current

        Pose2D target = new Pose2D(pos.x + dx, pos.y + dy, pos.h + dh);

        while (Math.abs(target.x - pos.x) > 0.5 ||
                Math.abs(target.y - pos.y) > 0.5 ||
                Math.abs(target.h - pos.h) > 5
        ) {  // I think this while loop is causing an exit error
            pos = otos.getPosition();

            double x = Range.clip(xController.calculate(pos.x, target.x), -1., 1.);  // strafe
            double y = Range.clip(yController.calculate(pos.y, target.y), -1., 1.);  // forward
            double h = Range.clip(headingController.calculate(pos.h, target.h), -1., 1.);  // turn

            setPowers(y + x + h, y - x - h, y - x + h, y + x - h);

            telemetry.addLine(String.format("x | currently at %f, targeting %f, power %f", pos.x, target.x, x));
            telemetry.addLine(String.format("y | currently at %f, targeting %f, power %f", pos.y, target.y, y));
            telemetry.addLine(String.format("h | currently at %f, targeting %f, power %f", pos.h, target.h, h));
            telemetry.update();

            System.out.printf("x | currently at %f, targeting %f, power %f\n", pos.x, target.x, x);
            System.out.printf("y | currently at %f, targeting %f, power %f\n", pos.y, target.y, y);
            System.out.printf("h | currently at %f, targeting %f, power %f\n", pos.h, target.h, h);
        }
    }
}
