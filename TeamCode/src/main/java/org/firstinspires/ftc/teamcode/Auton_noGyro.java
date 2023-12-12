package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//note: cannot set very accurate positions without gyro

@Autonomous(name = "AutonomousNoGyro")
public class Auton_noGyro extends Base {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sleep(500);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        driveFor(1, 0, 0, 3000, timer);
    }

    public void driveFor(double drive, double turn, double strafe, double milliseconds, ElapsedTime timer) {
        turn *= 1.1;
        timer.reset();
        double denominator = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);
        double topLeftPow = (drive + turn + strafe) / denominator;
        double backLeftPow = (drive + turn - strafe) / denominator;
        double topRightPow = (drive - turn - strafe) / denominator;
        double backRightPow = (drive - turn + strafe) / denominator;
        while (timer.milliseconds() <= milliseconds) {
            setDrivePowers(topLeftPow, backLeftPow, topRightPow, backRightPow);
        }
    }

    public void setDrivePowers(double backLeftPow, double topLeftPow, double backRightPow, double topRightPow) {
        backLeft.setPower(backLeftPow);
        topLeft.setPower(topLeftPow);
        backLeft.setPower(backLeftPow);
        topRight.setPower(topRightPow);
    }
}
