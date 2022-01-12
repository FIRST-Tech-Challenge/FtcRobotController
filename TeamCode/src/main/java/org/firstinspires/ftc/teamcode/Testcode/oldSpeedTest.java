package org.firstinspires.ftc.teamcode.Testcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Old Speed Test", group="Iterative Opmode")
public class oldSpeedTest extends OpMode {
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx rearLeft = null;
    private DcMotorEx rearRight = null;

    double testVelocity = 1000; // Robot velocity
    final double encoderResolution = 537.7; // Ticks per revolution
    final double wheelDiameter = 96; // mm
    final double wheelDist = inToCm(Math.hypot(13, 16)); // cm

    final double mmPerTick = (Math.PI*wheelDiameter)/encoderResolution;
    final double rotDist = Math.PI*wheelDist;
    double lastEncoderPosition;


    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        rearLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        rearRight = hardwareMap.get(DcMotorEx.class, "rightRear");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        rearLeft.setDirection(DcMotorEx.Direction.REVERSE);

        lastEncoderPosition = frontLeft.getCurrentPosition();


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
    }


    @Override
    public void loop() {
        // dpad movement events
        if(gamepad1.dpad_up) {
            drive(testVelocity);
        } else if(gamepad1.dpad_down) {
            drive(0-testVelocity);
        } else if(gamepad1.dpad_left) {
            strafe(testVelocity);
        } else if(gamepad1.dpad_right) {
            strafe(0-testVelocity);
        } else {
            drive(0);
        }

        // encoder offset
        if(gamepad1.a) {
            lastEncoderPosition = frontLeft.getCurrentPosition();
        }
        telemetry.addData("Encoder Offset", frontLeft.getCurrentPosition() - lastEncoderPosition);

        // change velocity
        if(gamepad1.right_bumper && testVelocity < 800) {
            testVelocity += 1;
        } else if(gamepad1.left_bumper && testVelocity > 0) {
            testVelocity -= 1;
        }

        /*  Test Movements  */
        // square w/ rotation
        if(gamepad1.x) {
            driveDist(testVelocity, ftToCm(2));
        }

        // square w/o rotation
        if(gamepad1.y) {
            driveDist(testVelocity,  ftToCm(2));
            strafeDist(testVelocity, ftToCm(2));
            driveDist(testVelocity, 0-ftToCm(2));
            strafeDist(testVelocity, 0-ftToCm(2));
        }

        // full rotation
        if(gamepad1.b) {
            strafeDist(testVelocity, ftToCm(2));
        }

    }

    // drive forward/backward
    void drive(double velocity) {
        frontLeft.setVelocity(velocity);
        frontRight.setVelocity(velocity);
        rearLeft.setVelocity(velocity);
        rearRight.setVelocity(velocity);
    }
    void driveDist(double velocity, double distance /* cm */) {
        // calculate ticks
        double mm = cmToMm(distance);
        double ticks = mm/mmPerTick;
        // calculate time in seconds
        double time = ticks/velocity;

        // drive
        drive(velocity);
        try {
            TimeUnit.SECONDS.sleep((long) time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive(0);
    }

    // strafe
    void strafe(double velocity) {
        frontLeft.setVelocity(-velocity);
        frontRight.setVelocity(velocity);
        rearLeft.setVelocity(velocity);
        rearRight.setVelocity(-velocity);
    }
    // https://www.reddit.com/r/FTC/comments/ltzywb/what_is_the_speed_ratio_of_the_robot_strafing_vs/
    void strafeDist(double velocity, double distance /* cm */) {
        // calculate ticks
        double mm = cmToMm(distance)*1.6;
        double ticks = mm/mmPerTick;
        // calculate time in seconds
        double time = ticks/velocity;

        // strafe
        strafe(velocity);
        try {
            TimeUnit.SECONDS.sleep((long) time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        strafe(0);
    }

    // rotate
    void rot(double velocity) {
        frontLeft.setVelocity(-velocity);
        frontRight.setVelocity(velocity);
        rearLeft.setVelocity(-velocity);
        rearRight.setVelocity(velocity);
    }
    void rotDeg(double velocity, double deg) {
        // calculate distance for wheel turn
        double distance = (rotDist/365)*deg; // cm
        double mm = cmToMm(distance);
        // calculate ticks
        double ticks = mm/mmPerTick;
        // calculate time in seconds
        double time = ticks/velocity;

        // rotate
        rot(velocity);
        try {
            TimeUnit.SECONDS.sleep((long) time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        rot(0);
    }


    @Override
    public void stop() {
        drive(0);
        strafe(0);
        rot(0);
    }


    // Distance Conversions
    double inToCm(double in) {
        return in*2.54;
    }
    double ftToCm(double ft) {
        return ft*12*2.54;
    }
    double cmToMm(double cm) {
        return cm*10;
    }
}