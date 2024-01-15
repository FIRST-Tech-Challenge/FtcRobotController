package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="ServerPosTest", group = "GRRRRR")
public class ServoPosTest extends LinearOpMode {
    Servo planeRaise;
    Servo clawServo;
    Servo clawArm;
    Servo clawAngle;
    Servo cameraTurning;
    Servo outtakeHook;
    Servo outtakeRotation;
    Servo backSlideServo;

    double[] servoPos = {.5,.5,.5,.5,.5,.5,.5,.5,};
    int target = 0;

    @Override
    public void runOpMode() {

        planeRaise = hardwareMap.servo.get("planeRaise");
        clawServo = hardwareMap.servo.get("clawServo");
        clawArm = hardwareMap.servo.get("clawArm");
        clawAngle = hardwareMap.servo.get("clawAngle");
        cameraTurning = hardwareMap.servo.get("cameraTurning");
        outtakeHook = hardwareMap.servo.get("outtakeHook");
        outtakeRotation = hardwareMap.servo.get("outtakeRotation");
        backSlideServo = hardwareMap.servo.get("backSlideServo");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && servoPos[target]< 1) {
                if (target==9 || target ==8){
                    servoPos[8]+=.005;
                    servoPos[9] += .005;
                } else {
                    servoPos[target] += .005;
                }

            } else if (gamepad1.b && servoPos[target] > 0) {
                if (target==9 || target==8){
                    servoPos[8]-=.005;
                    servoPos[9] -= .005;
                } else {
                    servoPos[target] -= .005;
                }
            }

            if (gamepad1.x) {
                target++;
                if (target == 10){
                    target = 0;
                }
                sleep(300);
            } else if (gamepad1.y) {
                target--;
                if (target == -1){
                    target = 9;
                }
                sleep(300);
            }

            planeRaise.setPosition(servoPos[0]);
            clawServo.setPosition(servoPos[1]);
            clawArm.setPosition(servoPos[2]);
            clawAngle.setPosition(servoPos[3]);
            cameraTurning.setPosition(servoPos[4]);
            outtakeHook.setPosition(servoPos[5]);
            outtakeRotation.setPosition(servoPos[6]);
            backSlideServo.setPosition(servoPos[7]);

            telemetry.addData("planeRaise",servoPos[1]);
            telemetry.addData("clawServo",servoPos[2]);
            telemetry.addData("clawArm",servoPos[3]);
            telemetry.addData("clawAngle",servoPos[4]);
            telemetry.addData("cameraTurning",servoPos[5]);
            telemetry.addData("outtakeHook",servoPos[6]);
            telemetry.addData("outtakeRotation",servoPos[7]);
            telemetry.addData("backSlideServo",servoPos[8]);

            telemetry.addData("Currently Editing: ", target);

            telemetry.update();
        }
    }
}