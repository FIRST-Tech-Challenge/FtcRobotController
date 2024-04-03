package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "Test Servo", group = "LinearOpMode")
public class testServo extends LinearOpMode {
    Servo trapdoorServo, droneServo;
    ElapsedTime trapdoorTime;
    ElapsedTime droneTime;
    boolean mailboxOpen = false;
    boolean droneToggle = false;


    @Override
    public void runOpMode() {
        trapdoorServo = hardwareMap.get(Servo.class, "trapdoorServo");
        droneServo = hardwareMap.get(Servo.class, "droneServo");
        trapdoorTime = new ElapsedTime();
        droneTime = new ElapsedTime();

        waitForStart();
        while(opModeIsActive()){
            trapdoor(gamepad2.x, trapdoorTime);
            launchDrone(gamepad2.y, droneTime);
        }
    }

    public void trapdoor(boolean button, ElapsedTime time) {
        if (button && time.time() > .50 && !mailboxOpen) {
            mailboxOpen = true;
            time.reset();
            trapdoorServo.setPosition(1.0);

        }
        else if (button && time.time() > .50 && mailboxOpen) {
            mailboxOpen = false;
            time.reset();
            trapdoorServo.setPosition(0.0);

        }
    }

    public void launchDrone(boolean button, ElapsedTime time) {
        if (button && time.time() > .75 && !droneToggle) {
            droneToggle = true;
            time.reset();
            droneServo.setPosition(0.0);

        } else if (button && time.time() > .75 && droneToggle) {
            droneToggle = false;
            time.reset();
            droneServo.setPosition(1.0);
        }
    }
}
