package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "servoTesting", group = "Testing")
public class servoTesting extends LinearOpMode {
    // declare buttons
    Button handlerDPad_Left;
    Button handlerDPad_Up;
    Button handlerDPad_Down;
    Button handlerRightTrigger;
    Servo clawYaw;

    double clawYawIntake, clawYawLeftHorizontal, clawYawLeftSlantedUp, clawYawLeftSlantedDown,  clawYawRightHorizontal, clawYawRightSlantedUp, clawYawRightSlantedDown;


    public void Setup() {

        clawYaw = hardwareMap.servo.get("clawYaw");
        // map buttons
        handlerDPad_Left = new Button(gamepad2, Button.NAME.DPAD_LEFT);
        handlerDPad_Down = new Button(gamepad2, Button.NAME.DPAD_DOWN);
        handlerDPad_Up = new Button(gamepad2, Button.NAME.DPAD_UP);
        handlerRightTrigger = new Button(gamepad2, Button.NAME.RIGHT_TRIGGER);

        clawYawIntake = 0.5;
        // Slanted is 60 degrees, allows us to drop pixels vertically for mosaics
        clawYawLeftSlantedUp = 1;
        clawYawLeftHorizontal = clawYawIntake+(5.0/3);
        clawYawLeftSlantedDown = clawYawLeftHorizontal+0.2;

        clawYawRightSlantedUp = 0;
        clawYawRightHorizontal = clawYawIntake-(5.0/3);
        clawYawRightSlantedDown = clawYawRightHorizontal-0.2;

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        waitForStart();
        while (opModeIsActive()) {
            updateButtons();

            if(handlerDPad_Left.Pressed()){
                if(handlerRightTrigger.Pressed()){
                    clawYaw.setPosition(clawYawRightHorizontal);
                }
                else{
                    clawYaw.setPosition(clawYawLeftHorizontal);
                }
            }

            if(handlerDPad_Down.Pressed()){
                if(handlerRightTrigger.Pressed()){
                    clawYaw.setPosition(clawYawRightSlantedUp);
                }
                else{
                    clawYaw.setPosition(clawYawLeftSlantedDown);
                }
            }

            if(handlerDPad_Up.Pressed()){
                if(handlerRightTrigger.Pressed()){
                    clawYaw.setPosition(clawYawRightSlantedDown);
                }
                else{
                    clawYaw.setPosition(clawYawLeftSlantedUp);

                }
            }
        }

    }

    private void updateButtons() {
        handlerDPad_Left.updateButton(gamepad1);
        handlerDPad_Up.updateButton(gamepad1);
        handlerDPad_Down.updateButton(gamepad1);
    }
}
