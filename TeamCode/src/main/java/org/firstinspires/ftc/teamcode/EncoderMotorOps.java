package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class EncoderMotorOps {
    private Robot robot;
    private DcMotorEx motor = null;
    private boolean verbose = false;
    private int motor_ticks = 1425;
    private int rev_ticks = 250;
    private boolean inAutoOp = false;
    private int auto_ticks  = 0;
    private int tolerance = 10;
    private int pos_min = 0;
    private int pos_max = 0;
    private double auto_power = 0.5;
    private double cur_manual_power = 0.5;
    private int cur_position = 0;

    public EncoderMotorOps(Robot robot, DcMotorEx motor, int pos_min, int pos_max, double auto_power, boolean verbose)
    {
        this.robot = robot;
        this.motor = motor;
        this.pos_min = pos_min;
        this.pos_max = pos_max;
        this.auto_power = auto_power;
        this.verbose = verbose;

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setTargetPositionTolerance(5);
    }
    public void logUpdate()
    {
        if (!verbose)
            return;
        robot.telemetry.addData("AutoOp: ", inAutoOp);
        robot.telemetry.addData("AutoOp Ticks: ", auto_ticks);
        robot.telemetry.addData("Current Position: ", cur_position);
        robot.telemetry.addData("AutoOp Power: ", auto_power);
        robot.telemetry.addData("ManualOp: ", !inAutoOp);
        robot.telemetry.addData("Manual Power: ", cur_manual_power);
        robot.telemetry.update();

    }
    public void log(String s, Double d)
    {
        if (verbose) { robot.telemetry.addData(s, d); }
    }
    private boolean limitCheck(boolean up) {
        cur_position = motor.getCurrentPosition();
        // NEW
        if (cur_position < 0) {
            // If the encoder is negative, this could be due slipping, reset the encoder to 0
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
        if (!up && cur_position <= 0) {
            //log("Reached Bottom: Stopping the Slider", 0.0);
            return true;
        }
        /*
        if (up && cur_position >= pos_max) {
            //log("Reached Max Height: Stopping the Slider", (double)pos_max);
            return true;
        }
        */
        return false;
    }

    public void manualOp(double power)
    {
        // If in Auto Operation cancel it
        if (inAutoOp) {
            inAutoOp = false;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }

        // NEW
        //if (limitCheck(power < 0)) {
        //  motor.setPower(0);
          //  return;
        //}
        cur_position = motor.getCurrentPosition();
        motor.setPower(-power);
        cur_manual_power = -power;
    }

    public void autoOp(int target)
    {
        cur_position = motor.getCurrentPosition();
        if (in_tolerance(cur_position, target)) {
            return;
        }
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(-auto_power);
        inAutoOp = true;
        auto_ticks = target;
    }

    private boolean in_tolerance(int ticks, int target)
    {
        if (Math.abs(Math.abs(ticks) - Math.abs(target)) <= tolerance) {
            return true;
        }
        return false;
    }

    public void autoOpCompletionCheck()
    {
        if (!inAutoOp) {
            return;
        }
        cur_position = motor.getCurrentPosition();
        if (in_tolerance(cur_position, auto_ticks)) {
            inAutoOp = false;
            // set the motor back to manual control
            if (cur_position < 0) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
    }
    public void manualDefaultStop() {
        if (!inAutoOp) {
            motor.setPower(0);
        }
    }
}
