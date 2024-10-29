package org.firstinspires.ftc.teamcode.JackBurr.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TankDriveUsingRegularControls extends OpMode {
    public enum DirectionState {
        FORWARD,
        BACKWARD,
        TURN_LEFT,
        TURN_RIGHT,
        STOP
    }
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DirectionState state = DirectionState.STOP;


    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_y < 0){
            frontLeft.setPower(gamepad1.left_stick_y);
            backLeft.setPower(gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.left_stick_y);
            backRight.setPower(-gamepad1.left_stick_y);
            state = DirectionState.FORWARD;
        }
        else if(gamepad1.left_stick_y > 0){
            frontLeft.setPower(gamepad1.left_stick_y);
            backLeft.setPower(gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.left_stick_y);
            backRight.setPower(-gamepad1.left_stick_y);
            state = DirectionState.BACKWARD;
        }
        else if(gamepad1.right_stick_x < 0){
            frontLeft.setPower(-gamepad1.right_stick_x);
            backLeft.setPower(-gamepad1.right_stick_x);
            frontRight.setPower(-gamepad1.right_stick_x);
            backRight.setPower(-gamepad1.right_stick_x);
            state = DirectionState.TURN_LEFT;
        }
        else if(gamepad1.right_stick_x > 0){
            frontLeft.setPower(gamepad1.right_stick_x);
            backLeft.setPower(gamepad1.right_stick_x);
            frontRight.setPower(gamepad1.right_stick_x);
            backRight.setPower(gamepad1.right_stick_x);
            state = DirectionState.TURN_RIGHT;
        }
        else {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            state = DirectionState.TURN_LEFT;
            state = DirectionState.STOP;
        }
        if(gamepad1.left_bumper && gamepad1.right_bumper){
            requestOpModeStop();
        }
    }
}
