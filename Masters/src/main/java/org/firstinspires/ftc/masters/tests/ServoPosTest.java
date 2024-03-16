//package org.firstinspires.ftc.masters.tests;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@Config
//@TeleOp(name="ServerPosTest", group = "GRRRRR")
//public class ServoPosTest extends LinearOpMode {
//    Servo plane;
//    Servo clawServo;
//    Servo clawArm;
//    Servo clawAngle;
//    Servo outtakeHook;
//    Servo outtakeRotation;
//    Servo outtakeMovement;
//
//    double[] servoPos = {.5,.5,.5,0,.5,.5,.5,};
//    int target = 0;
//
//    @Override
//    public void runOpMode() {
//        plane = hardwareMap.servo.get("planeRaise");
//        clawServo = hardwareMap.servo.get("clawServo");
//        clawArm = hardwareMap.servo.get("clawArm");
//        clawAngle = hardwareMap.servo.get("clawAngle");
//        outtakeHook = hardwareMap.servo.get("outtakeHook");
//        outtakeRotation = hardwareMap.servo.get("outtakeRotation");
//        outtakeMovement = hardwareMap.servo.get("backSlideServo");
//
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            if (gamepad1.a && servoPos[target]< 1) {
//                servoPos[target] += .005;
//            } else if (gamepad1.b && servoPos[target] > 0) {
//                servoPos[target] -= .005;
//            }
//
//            if (gamepad1.x) {
//                target++;
//                if (target == 10){
//                    target = 0;
//                }
//                sleep(300);
//            } else if (gamepad1.y) {
//                target--;
//                if (target == -1){
//                    target = 9;
//                }
//                sleep(300);
//            }
//
//            plane.setPosition(servoPos[0]);
//            clawServo.setPosition(servoPos[1]);
//            clawArm.setPosition(servoPos[2]);
//            clawAngle.setPosition(servoPos[3]);
//            outtakeHook.setPosition(servoPos[4]);
//            outtakeRotation.setPosition(servoPos[5]);
//            outtakeMovement.setPosition(servoPos[6]);
//
//            telemetry.addData("plane",servoPos[0]);
//            telemetry.addData("clawServo",servoPos[1]);
//            telemetry.addData("clawArm",servoPos[2]);
//            telemetry.addData("clawAngle",servoPos[3]);
//            telemetry.addData("outtakeHook",servoPos[4]);
//            telemetry.addData("outtakeRotation",servoPos[5]);
//            telemetry.addData("outtakeMovement",servoPos[6]);
//
//            telemetry.addData("Currently Editing: ", target);
//
//            telemetry.update();
//        }
//    }
//}