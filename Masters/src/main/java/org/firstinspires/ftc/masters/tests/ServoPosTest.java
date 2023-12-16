package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="ServerPosTest", group = "GRRRRR")
public class ServoPosTest extends LinearOpMode {
    Servo planeLaunch;
    Servo planeRaise;
    Servo clawServo;
    Servo clawArm;
    Servo clawAngle;
    Servo cameraTurning;
    Servo outtakeHook;
    Servo outtakeRotation;
    Servo outtakeMovementRight;
    Servo outtakeMovementLeft;

    double[] servoPos = {.5,.5,.5,.5,.5,.5,.5,.5,.5,.5};
    int target = 0;

    @Override
    public void runOpMode() {

        planeLaunch = hardwareMap.servo.get("planeLaunch");
        planeRaise = hardwareMap.servo.get("planeRaise");
        clawServo = hardwareMap.servo.get("clawServo");
        clawArm = hardwareMap.servo.get("clawArm");
        clawAngle = hardwareMap.servo.get("clawAngle");
        cameraTurning = hardwareMap.servo.get("cameraTurning");
        outtakeHook = hardwareMap.servo.get("outtakeHook");
        outtakeRotation = hardwareMap.servo.get("outtakeRotation");
        outtakeMovementRight = hardwareMap.servo.get("outtakeMovementRight");
        outtakeMovementLeft = hardwareMap.servo.get("outtakeMovementLeft");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && servoPos[target]< 1) {
                servoPos[target]+=.005;
            } else if (gamepad1.b && servoPos[target] > 0) {
                servoPos[target]-=.005;
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

            planeLaunch.setPosition(servoPos[0]);
            planeRaise.setPosition(servoPos[1]);
            clawServo.setPosition(servoPos[2]);
            clawArm.setPosition(servoPos[3]);
            clawAngle.setPosition(servoPos[4]);
            cameraTurning.setPosition(servoPos[5]);
            outtakeHook.setPosition(servoPos[6]);
            outtakeRotation.setPosition(servoPos[7]);
            outtakeMovementRight.setPosition(servoPos[8]);
            outtakeMovementLeft.setPosition(servoPos[9]);

            telemetry.addData("planeLaunch",servoPos[0]);
            telemetry.addData("planeRaise",servoPos[1]);
            telemetry.addData("clawServo",servoPos[2]);
            telemetry.addData("clawArm",servoPos[3]);
            telemetry.addData("clawAngle",servoPos[4]);
            telemetry.addData("cameraTurning",servoPos[5]);
            telemetry.addData("outtakeHook",servoPos[6]);
            telemetry.addData("outtakeRotation",servoPos[7]);
            telemetry.addData("outtakeMovementRight",servoPos[8]);
            telemetry.addData("outtakeMovementLeft",servoPos[9]);

            telemetry.addData("Currently Editing: ", target);

            telemetry.update();
        }
    }
}