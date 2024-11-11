package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTest")
public class MotorTest extends OpMode {
    private DcMotor rightFront, rightRear, leftFront, leftRear;

    @Override
    public void init() {
        this.rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        this.rightRear = hardwareMap.get(DcMotor.class,"rightRear");
        this.leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        this.leftRear = hardwareMap.get(DcMotor.class,"leftRear");
        this.rightFront.setDirection(DcMotor.Direction.REVERSE);
        this.rightRear.setDirection(DcMotor.Direction.REVERSE);
        this.leftFront.setDirection(DcMotor.Direction.FORWARD);
        this.leftRear.setDirection(DcMotor.Direction.FORWARD);
        this.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        telemetry.addData("rightFront", this.gamepad1.dpad_up);
        if (this.gamepad1.dpad_up) {
            this.rightFront.setPower(0.25);
        } else {
            this.rightFront.setPower(0);
        }
        telemetry.addData("rightRear", this.gamepad1.dpad_right);
        if (this.gamepad1.dpad_right) {
            this.rightRear.setPower(0.25);
        } else {
            this.rightRear.setPower(0);
        }
        telemetry.addData("leftRear", this.gamepad1.dpad_down);
        if (this.gamepad1.dpad_down) {
            this.leftRear.setPower(0.25);
        } else {
            this.leftRear.setPower(0);
        }
        telemetry.addData("leftFront", this.gamepad1.dpad_left);
        if (this.gamepad1.dpad_left) {
            this.leftFront.setPower(0.25);
        } else {
            this.leftFront.setPower(0);
        }
        telemetry.update();
    }
}
