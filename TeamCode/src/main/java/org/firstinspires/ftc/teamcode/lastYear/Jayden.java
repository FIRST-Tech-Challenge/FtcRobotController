package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class Jayden extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        telemetry.addData("Initialized", "Ready to start");
        waitForStart();

        forward(1, 1000);
        sleep(1000);

        backward(1, 1000);
        sleep(1000);

        strafeRight(1, 1000);
        sleep(1000);

        strafeLeft(1, 1000);
        sleep(1000);

        spinLeft(1, 1000);
        sleep(1000);

        spinRight(1, 1000);
        sleep(1000);
    }

    public void spinLeftRaw(double power) {
        frontRight.setPower(power*-1);
        backRight.setPower(power*-1);
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    public void spinRightRaw(double power) {
        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power*-1);
        backLeft.setPower(power*-1);
    }

    public void strafeLeftRaw(double power) {
        frontRight.setPower(power);
        backRight.setPower(power*-1);
        frontLeft.setPower(power*-1);
        backLeft.setPower(power);
    }

    public void strafeRightRaw(double power) {
        frontRight.setPower(power*-1);
        backRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power*-1);
    }

    public void backwardRaw(double power) {
        frontRight.setPower(power*-1);
        backRight.setPower(power*-1);
        frontLeft.setPower(power*-1);
        backLeft.setPower(power*-1);
    }

    public void forwardRaw(double power) {
        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    public void forwardRaw() {
        forwardRaw(1);
    }

    public void strafeLeftRaw() {
        strafeLeftRaw(1);
    }

    public void strafeRightRaw() {
        strafeRightRaw(1);
    }

    public void backwardRaw() {
        backwardRaw(1);
    }

    public void spinLeftRaw() {
        spinLeftRaw(1);
    }

    public void spinRightRaw() {
        spinRightRaw(1);
    }

    public void Run_with_encoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void run_using_encoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void run_to_position() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stop_and_reset_encoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void forward(double power, int ticks) {
        stop_and_reset_encoders();
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        run_to_position();
        while (frontLeft.isBusy()) {
            forwardRaw(power);
        }
    }

    public void backward(double power, int ticks) {
        ticks = (ticks*-1);
        stop_and_reset_encoders();
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        run_to_position();
        while (frontLeft.isBusy()) {
            backwardRaw(power);
        }
    }

    public void strafeRight(double power, int ticks) {
        stop_and_reset_encoders();
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks*-1);
        backLeft.setTargetPosition(ticks*-1);
        backRight.setTargetPosition(ticks);
        run_to_position();
        while (frontLeft.isBusy()) {
            strafeRightRaw(power);
        }
    }

    public void strafeLeft(double power, int ticks) {
        stop_and_reset_encoders();
        frontLeft.setTargetPosition(ticks*-1);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks*-1);
        run_to_position();
        while (frontLeft.isBusy()) {
            strafeLeftRaw(power);
        }
    }

    public void spinLeft(double power, int ticks) {
        stop_and_reset_encoders();
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks*-1);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks*-1);
        run_to_position();
        while (frontLeft.isBusy()) {
            spinLeftRaw(power);
        }
    }

    public void spinRight(double power, int ticks) {
        stop_and_reset_encoders();
        frontLeft.setTargetPosition(ticks*-1);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks*-1);
        backRight.setTargetPosition(ticks);
        run_to_position();
        while (frontLeft.isBusy()) {
            spinRightRaw(power);
        }
    }
}
