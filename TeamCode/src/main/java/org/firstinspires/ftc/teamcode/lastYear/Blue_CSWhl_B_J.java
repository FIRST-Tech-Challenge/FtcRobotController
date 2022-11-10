package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Blue_CSWhl (Blocks to Java)")
@Disabled
public class Blue_CSWhl_B_J extends LinearOpMode {

    private DcMotor caroswl;
    private DcMotor front_left;
    private DcMotor back_right;
    private DcMotor front_right;
    private DcMotor back_left;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station
     */
    @Override
    public void runOpMode() {
        caroswl = hardwareMap.get(DcMotor.class, "caroswl");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            strafe(-400, -0.5);
            forward(-1500, -0.5);
            strafe(400, 0.5);
            caroswl.setPower(1);
            sleep(1000);
            caroswl.setPower(0);
            strafe(-100, -0.5);
            forward(1000, 0.5);
            strafe(300, 0.5);
            forward(1000, 0.5);
        }
    }

    /**
     * Describe this function...
     */
    private void Run_with_encoder() {
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void run_using_encoder() {
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void run_to_position() {
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void stop_and_reset_encoder() {
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void forward(int distance, double power) {
        stop_and_reset_encoder();
        front_left.setTargetPosition(distance * -1);
        back_right.setTargetPosition(distance);
        front_right.setTargetPosition(distance);
        back_left.setTargetPosition(distance * -1);
        run_to_position();
        while (front_left.isBusy()) {
            front_left.setPower(power * -1);
            back_right.setPower(power);
            front_right.setPower(power);
            back_left.setPower(power * -1);
        }
    }

    /**
     * Describe this function...
     */
    private void spin(double distance, double power) {
        stop_and_reset_encoder();
        front_left.setTargetPosition((int) (distance * -1));
        back_right.setTargetPosition((int) (distance * -1));
        front_right.setTargetPosition((int) (distance * -1));
        back_left.setTargetPosition((int) (distance * -1));
        run_to_position();
        while (front_left.isBusy()) {
            front_left.setPower(power * -1);
            back_right.setPower(power * -1);
            front_right.setPower(power * -1);
            back_left.setPower(power * -1);
        }
    }

    /**
     * Describe this function...
     */
    private void strafe(int distance, double power) {
        stop_and_reset_encoder();
        front_left.setTargetPosition(distance * -1);
        back_right.setTargetPosition(distance * -1);
        front_right.setTargetPosition(distance);
        back_left.setTargetPosition(distance);
        run_to_position();
        while (front_left.isBusy()) {
            front_left.setPower(power * -1);
            back_right.setPower(power * -1);
            front_right.setPower(power);
            back_left.setPower(power);
        }
    }
}