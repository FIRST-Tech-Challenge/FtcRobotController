package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.Team6200.Movement;


import java.util.HashMap;

@TeleOp
public class Op6200Driven extends OpMode {
    HashMap<String, HardwareDevice> hardware = new HashMap<>();
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private Movement movement;
    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("front_right");
        leftFront = hardwareMap.dcMotor.get("front_left");
        rightBack = hardwareMap.dcMotor.get("back_right");
        leftBack = hardwareMap.dcMotor.get("back_left");
        hardware.put("frontRight", rightFront);
        hardware.put("frontLeft", leftFront);
        hardware.put("backRight", rightBack);
        hardware.put("backLeft", leftBack);
        movement = new Movement(hardware);
    }

    @Override
    public void loop() {

        movement.processJoystick(gamepad1.left_stick_x, 0 - gamepad1.left_stick_y, gamepad1.right_stick_x);

        //manual movement for linear slide
        int lmotorpos = lmotor.getCurrentPosition();
        if(gamepad1.right_trigger != 0)
        {
            lmotor.setTargetPosition(lmotorpos + 60);
            lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lmotor.setPower(0.8);
            telemetry.addData("Current position", lmotor.getCurrentPosition());

        }

        if(gamepad1.left_trigger != 0)
        {

            lmotor.setTargetPosition(lmotorpos - 60);
            lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lmotor.setPower(0.8);
            telemetry.addData("Current position", lmotor.getCurrentPosition());

        }
    }
}
