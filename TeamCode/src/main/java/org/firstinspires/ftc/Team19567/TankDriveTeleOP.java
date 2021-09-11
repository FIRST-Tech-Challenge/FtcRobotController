package org.firstinspires.ftc.Team19567;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TankTeleOP", group="Iterative Opmode")
public class TankDriveTeleOP extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightDC = null;
    private DcMotor leftDC = null;
    private boolean slowMode = false;
    private double acc = 1.0;
    private Servo servo1 = null;
    private boolean isServoMaxed = false;

    @Override
    public void init() {

        leftDC  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightDC = hardwareMap.get(DcMotor.class, "rightMotor");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        leftDC.setDirection(DcMotor.Direction.FORWARD);
        rightDC.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Awaiting Start");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("Status", "Started");
        telemetry.update();
        runtime.reset();
    }

    @Override
    public void loop() {

        final double leftSpeed = Range.clip(-gamepad1.left_stick_y + gamepad1.right_stick_x, -1.0, 1.0) ;
        final double rightSpeed = Range.clip(-gamepad1.left_stick_y - gamepad1.right_stick_x, -1.0, 1.0) ;

        if(gamepad1.x) slowMode = !slowMode;
        if(gamepad1.a) isServoMaxed = !isServoMaxed;

        if(slowMode) acc = 0.3;
        else acc = 1.0;
        if(isServoMaxed) servo1.setPosition(1.0);
        else servo1.setPosition(0.1);

        leftDC.setPower(leftSpeed);
        rightDC.setPower(rightSpeed);

        telemetry.addData("Status", "Looping");
        telemetry.addData("Runtime", runtime.toString() + " Milliseconds");
        telemetry.addData("DCMotors", "Left (%.2f), Right (%.2f)", leftSpeed, rightSpeed);
        telemetry.addData("Servos", "servo1 (%.2f)", servo1.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}