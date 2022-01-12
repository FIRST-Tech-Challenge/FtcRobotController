package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Speed Test", group="Iterative Opmode")
public class MovementTest extends OpMode {
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx rearLeft = null;
    private DcMotorEx rearRight = null;

    final double encoderResolution = 537.7; // Ticks per revolution
    final double wheelDiameter = 96; // mm
    final double rotationDistance = 59.436; // cm
    final double strafeModifier = 1.125;
    final double mmPerTick = (Math.PI*wheelDiameter)/encoderResolution;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        rearLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        rearRight = hardwareMap.get(DcMotorEx.class, "rightRear");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        rearLeft.setDirection(DcMotorEx.Direction.REVERSE);
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
    }


    @Override
    public void loop() {
        if(gamepad1.a) {
            drive(inToCm(6), 0.2);
            rotate(90, 0.4);
            strafe(inToCm(6), 0.2);
            drive(ftToCm(-1), 0.3);
        }

        if(gamepad1.x) {
            drive(ftToCm(2), 0.2);
            rotate(90, 0.4);
            drive(ftToCm(2), 0.2);
            rotate(90, 0.4);
            drive(ftToCm(2), 0.2);
            rotate(90, 0.4);
            drive(ftToCm(2), 0.2);
            rotate(90, 0.4);
        }

        if(gamepad1.y) {
            rotate(90, 0.1);
        }

        if(gamepad1.b) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);
        }
    }

    // drive forward/backward
    void drive(double cm, double power) {
        double ticks = cmToTick(cm);
        moveMotor(frontLeft, (int)ticks, power);
        moveMotor(frontRight, (int)ticks, power);
        moveMotor(rearLeft, (int)ticks, power);
        moveMotor(rearRight, (int)ticks, power);
        while(frontLeft.isBusy() ||  frontRight.isBusy() || rearLeft.isBusy() || rearRight.isBusy()) {}
    }

    void strafe(double cm, double power) {
        double ticks = -cmToTick(cm)*strafeModifier;
        moveMotor(frontLeft, (int)-ticks, power);
        moveMotor(frontRight, (int)ticks, power);
        moveMotor(rearLeft, (int)ticks, power);
        moveMotor(rearRight, (int)-ticks, power);
        while(frontLeft.isBusy() ||  frontRight.isBusy() || rearLeft.isBusy() || rearRight.isBusy()) {}
    }

    void rotate(double deg, double power) {
        double ticks = -cmToTick(rotationDistance/360*deg);
        moveMotor(frontLeft, (int)-ticks, power);
        moveMotor(frontRight, (int)ticks, power);
        moveMotor(rearLeft, (int)-ticks, power);
        moveMotor(rearRight, (int)ticks, power);
        while(frontLeft.isBusy() ||  frontRight.isBusy() || rearLeft.isBusy() || rearRight.isBusy()) {}
    }

    void moveMotor(DcMotor motor, int ticks, double power) {
        int postion = motor.getCurrentPosition();
        motor.setTargetPosition(postion + ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    // Unit Conversions
    double inToCm(double in) { return in*2.54; }
    double ftToCm(double ft) { return ft*12*2.54; }
    double cmToTick(double cm) { return cm*10/mmPerTick; }
}