package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private Servo leftWrist;
    private Servo rightWrist;
    private Servo leftGrabber;
    private Servo rightGrabber;

    private HorizontalSlide hSlide;

    private OpMode opMode;
    private Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;


    int maxFlipPosition = 525;
    int minFlipPosition = 0;


    public Intake(OpMode opMode, HorizontalSlide hSlide) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        this.hSlide = hSlide;
        leftWrist = opMode.hardwareMap.get(Servo.class, "leftWrist");
        rightWrist = opMode.hardwareMap.get(Servo.class, "rightWrist");
        leftGrabber = opMode.hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = opMode.hardwareMap.get(Servo.class, "rightGrabber");

        leftWrist.setPosition(0);
        rightWrist.setPosition(1);
        leftGrabber.setPosition(0.5);
        rightGrabber.setPosition(0.5);
    }

    public void checkInputs(
            Boolean wristDown,
            Boolean wristUp,
            Boolean wristHalf,
            Boolean grabberSuck,
            Boolean grabberSpit
    ) {
        if((hSlide.getPos() < maxFlipPosition)) {
            if (wristDown && (hSlide.getPos() > minFlipPosition) && !gamepad1.start) {
                wristDown();
            }
            else if (wristUp && !gamepad1.start) {
                wristUp();
            }
            else if (wristHalf) {
                wristHalf();
            }

        }
        else {
            telemetry.addData("Wrist", "Cannot move wrist down, horizontal slide is too high");
        }


//        else if (wristUp && (hSlide.getPos() < maxFlipPosition) && !gamepad1.start) {
//            wristUp();
//        }
//        else if (wristHalf && (hSlide.getPos() < maxFlipPosition)) {
//            wristHalf();
//        }

        if (grabberSuck) {
            grabberSuck();
        } else if (grabberSpit) {
            grabberSpit();
        } else {
            grabberOff();
        }

    }

    // SERVO CONTROLLER INFO: leftWrist:        left limit = 1, right limit = 0     rightWrist: left limit = 0, right limit = 1
    public void setWristPosition(double leftPosition, double rightPosition) {
        leftWrist.setPosition(leftPosition);
        rightWrist.setPosition(rightPosition);
    }

    public void setGrabberPosition(double leftPosition, double rightPosition) {
        leftGrabber.setPosition(leftPosition);
        rightGrabber.setPosition(rightPosition);
    }

    public void wristDown() {
        setWristPosition(1, 0);
    }

    public void wristUp() {
        setWristPosition(0, 1);
    }

    public void wristHalf() {
        setWristPosition(0.6, 0.4);
    }

    public void grabberSuck() {
        setGrabberPosition(0, 1);
    }

    public void grabberSpit() {
        setGrabberPosition(1, 0);
    }

    public void grabberOff() {
        setGrabberPosition(0.5, 0.5);
    }
}