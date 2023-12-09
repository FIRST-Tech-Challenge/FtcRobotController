package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Slider {
    private Robot robot;
    private Gamepad gamepad;
    private double normal_speed = 1.0;
    private double slow_speed = 1.0;
    private int max_height_ticks = 4000;
    private boolean verbose = true;
    private int motor_ticks = 1425;

    private int rev_ticks = 250;

    private boolean inAutoBottom = false;

    public Slider(Robot robot, Gamepad gamepad)
    {
        this.robot = robot;
        this.gamepad = gamepad;

        robot.motorSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Assume the starting is the bottom most position
        //robot.motorSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.motorSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorSlider.setTargetPositionTolerance(3);
    }
    private void log(String s, Double d)
    {
        if (verbose) {
            robot.telemetry.addData(s, d);
        }
    }
    private void logUpdate()
    {
        if (verbose) {
            robot.telemetry.update();
        }
    }

    private boolean softlimitCheck(boolean up) {
        int cur_position = robot.motorSlider.getCurrentPosition();
        log("Slider Current Position=", (double)cur_position);
        if (!up && cur_position <= 3) {
            log("Reached Bottom: Stopping the Slider", 0.0);
            robot.motorSlider.setPower(0);
            return true;
        }
        if (up && cur_position > max_height_ticks) {
            log("Reached Max Height: Stopping the Slider", 0.0);
            robot.motorSlider.setPower(0);
            return true;
        }
        return false;
    }

    private void moveOp(double power)
    {
        boolean limit_reached = softlimitCheck(power < 0);
        if (limit_reached) {
            robot.motorSlider.setPower(0);
            robot.telemetry.addData("Reached", "Limit");
            //robot.telemetry.update();
            //return;
        }
        //robot.motorSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.telemetry.addData("SliderPower:", power);
        robot.motorSlider.setPower(power);
    }

    private void goToBottom()
    {
        robot.motorSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorSlider.setTargetPosition(0);
        robot.motorSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorSlider.setPower(0.4);
        inAutoBottom = true;
    }

    public void move()
    {
        int cur_position = robot.motorSlider.getCurrentPosition();

        if (verbose) {
            robot.telemetry.addData("Slider Current Position=", cur_position);
        }
        if (cur_position <= 3) {
            inAutoBottom = false;
        }
        if (inAutoBottom) {
            return;
        }
        if (gamepad.left_stick_y != 0) {
            moveOp(gamepad.left_stick_y * normal_speed);
        } else if (gamepad.left_stick_x != 0) {
            moveOp(gamepad.left_stick_x * slow_speed);
        } if (gamepad.b) {
            goToBottom();
        } else {
            robot.motorSlider.setPower(0);
        }
        robot.telemetry.update();
    }
}
