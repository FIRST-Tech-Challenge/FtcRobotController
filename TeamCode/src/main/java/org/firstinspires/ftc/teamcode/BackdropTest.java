package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Backdrop;



@TeleOp (name="BackDropTest", group="Robot")

public class BackdropTest extends LinearOpMode {

    // initialising motors
    public Backdrop backdrop;
    private boolean isDpadUpPressed = false;
    private boolean isDpadDownPressed = false;
    private boolean isXPressed = false;


    @Override
    public void runOpMode() {
        backdrop = new Backdrop(this);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad2.dpad_up && !isDpadUpPressed) {
                backdrop.moveUp();
            }

            if (gamepad2.dpad_down && !isDpadDownPressed) {
                backdrop.moveDown();
            }

            if (gamepad2.x && !isXPressed) {
                backdrop.release();
            }


            isDpadUpPressed = gamepad2.dpad_up;
            isDpadDownPressed = gamepad2.dpad_down;
            isXPressed = gamepad2.x;
            sleep(50);
        }
    }
}