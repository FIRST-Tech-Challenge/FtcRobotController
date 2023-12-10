package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Arm {
    private Robot robot;
    private Gamepad gamepad;
    private double speed = 0.5;
    private int pos_pixel  = 1000;
    private boolean inAutoMove = false;
    private int target = 0;
    public Arm(Robot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
        robot.motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Assume the starting is the top most position
        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorArm.setTargetPositionTolerance(3);
    }

    private void goToPos(int pos) {
        robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorArm.setTargetPosition(pos);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorArm.setPower(0.4);
        //inAutoPixel = true;
    }

    private void goToPixel()
    {
        goToPos(pos_pixel);
    }

    private void foldArm() {
        goToPos(0);
    }

    private void moveOp(double power)
    {
        robot.motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorArm.setPower(power);
    }

    public void move()
    {
        int cur_position = robot.motorSlider.getCurrentPosition();
        if (inAutoMove && cur_position != target) {
            return;
        } else {
            inAutoMove = false;
        }
        if (gamepad.x) {
            foldArm();
        } else if (gamepad.y) {
            goToPixel();
        } else if (gamepad.right_stick_y != 0) {
            moveOp(- gamepad.right_stick_y * speed);
        } else {
            robot.motorArm.setPower(0);
        }
    }
}
