package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Slider {
    private Robot robot;
    private Gamepad gamepad;
    private double normal_speed = 0.5;
    private double slow_speed = 0.3;
    int max_height_ticks = 4000;
    boolean verbose = true;
    int motor_ticks = 1425;

    int rev_ticks = 250;

    public Slider(Robot robot, Gamepad gamepad)
    {
        this.robot = robot;
        this.gamepad = gamepad;
        // Run the slider with the encoder for setting the limits etc.
        robot.motorSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorSlider.setTargetPositionTolerance(3);
    }

    private boolean softlimit_check(boolean up) {
        int cur_position = robot.motorSlider.getCurrentPosition();
        if (verbose) {
            robot.telemetry.addData("Slider Current Position=", cur_position);
            robot.telemetry.update();
        }
        if (!up && cur_position <= 3) {
            robot.motorSlider.setPower(0);
            return true;
        }
        if (up && cur_position > max_height_ticks) {
            robot.motorSlider.setPower(0);
            return true;
        }
        return false;
    }

    private void moveOp(double power)
    {
        boolean limit_reached = softlimit_check(power > 0);
        if (limit_reached) {
            robot.telemetry.addData("Reached", "Limit");
            robot.telemetry.update();
            return;
        }
        robot.motorSlider.setPower(power);
    }
    private void moveOp(int ticks, double power)
    {
        boolean limit_reached = softlimit_check(power > 0);
        if (limit_reached) {
            robot.motorSlider.setPower(0);
            return;
        }
        int cur_position = robot.motorSlider.getCurrentPosition();
        if (power < 0) {
            ticks = -ticks;
        }

        robot.motorSlider.setTargetPosition(cur_position + ticks);
        robot.motorSlider.setPower(power);
    }

    private void moveBottom()
    {
        robot.motorSlider.setTargetPosition(0);
        robot.motorSlider.setPower(0.4);

    }

    public void move()
    {
        int cur_position = robot.motorSlider.getCurrentPosition();
        if (verbose) {
            robot.telemetry.addData("Slider Current Position=", cur_position);
            robot.telemetry.update();
        }
        if (gamepad.left_stick_y != 0) {
            moveOp(gamepad.left_stick_y * normal_speed);
        } else if (gamepad.left_stick_x != 0) {
            moveOp(10, gamepad.left_stick_x * slow_speed);
        } if (gamepad.b) {
            moveBottom();
        } else {
            robot.motorSlider.setPower(0);
        }
    }
}
