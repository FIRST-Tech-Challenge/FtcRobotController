package org.firstinspires.ftc.teamcode.PID;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class DriveTrain {
    private final DcMotor fLeft, fRight, bLeft, bRight;
    private final Odometry odometry;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;


    public DriveTrain(HardwareMap hardwareMap) {
        fLeft = hardwareMap.get(DcMotor.class, "fLeft");
        fRight = hardwareMap.get(DcMotor.class, "fRight");
        bLeft = hardwareMap.get(DcMotor.class, "bLeft");
        bRight = hardwareMap.get(DcMotor.class, "bRight");

        fLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.FORWARD);

        DcMotor[] motors = {fLeft, fRight, bLeft, bRight};
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        odometry = new Odometry(hardwareMap.get(SparkFunOTOS.class, "sensorOTOS"));
        xController = new PIDController(0.05);  // placeholder P values
        yController = new PIDController(0.05);
        headingController = new PIDController(0.05);
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

    public void move(double dx, double dy, double dh) {
        Pose2D pos = odometry.getPosition();  // current
        Pose2D target = new Pose2D(pos.x + dx, pos.y + dy, pos.h + dh);

        while (Math.abs(target.x - pos.x) > 0.5 && Math.abs(target.y - pos.y) > 0.5 && Math.abs(target.h - pos.h) > 5) {
            pos = odometry.getPosition();
            double x = Range.clip(xController.calculate(pos.x, target.x), -1., 1.);
            double y = Range.clip(yController.calculate(pos.y, target.y), -1., 1.);
            double h = Range.clip(headingController.calculate(pos.h, target.h), -1., 1.);
            setPowers(x - y - h, x + y + h, x + y - h, x - y + h);
        }
    }
}
