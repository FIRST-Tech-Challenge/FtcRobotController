package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Testing", group="Linear Opmode")

public class TestOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        CRServo lServo = hardwareMap.get(CRServo.class, "right_spinner");
        CRServo rServo = hardwareMap.get(CRServo.class, "left_spinner");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double spin = -gamepad1.left_stick_y;

            if (spin > .2){
                lServo.setPower(1);
                rServo.setPower(-1);
            }else if(spin < -.2){
                lServo.setPower(-.25);
                rServo.setPower(.25);
            }else{
                lServo.setPower(0);
                rServo.setPower(0);
            }
        }
    }

    public void waitTime(double time){
        runtime.reset();
        while(runtime.seconds()<time){
        }
    }
}