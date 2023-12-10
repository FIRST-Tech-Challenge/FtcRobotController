package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Hang {
    private Robot robot;
    private Gamepad gamepad;
    private double speed = 1.0;
    private int pos_pixel  = 1000;
    private boolean inAutoMove = false;
    private int target = 0;
    public Hang(Robot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
        robot.motorHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Assume the starting is the top most position
        robot.motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorHang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorHang.setTargetPositionTolerance(3);
    }

    private void goToPos(int pos) {
        robot.motorHang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorHang.setTargetPosition(pos);
        robot.motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorHang.setPower(0.4);
        //inAutoPixel = true;
    }

    private void moveOp(double power)
    {
        robot.motorHang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorHang.setPower(power);
    }

    public void operate()
    {
        int cur_position = robot.motorHang.getCurrentPosition();
        if (inAutoMove && cur_position != target) {
            return;
        } else {
            inAutoMove = false;
        }

        if (gamepad.dpad_up) {
            moveOp(speed);
        } else {
            robot.motorHang.setPower(0);
        }
    }
}
