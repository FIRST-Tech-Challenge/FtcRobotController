package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled

@TeleOp(name = "TUrTest")
//@Disabled

public class TurTest extends LinearOpMode {
    public void runOpMode() {

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();
//        Robot robot = new Robot(this, BasicChassis.ChassisType.VSLAM, false ,false);
        Servo intakeServo = this.hardwareMap.get(Servo.class, "IntakeServo");
        Servo intakeServo2 = this.hardwareMap.get(Servo.class, "IntakeServo2");
        Servo basketArmServo = this.hardwareMap.get(Servo.class, "basketActuationServo");
        Servo basketActuationServo = this.hardwareMap.get(Servo.class, "basketArmServo");
        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();
        double basketActuation = 0;
        double basketArm = 0;
        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        double diSTANCe=0;
        double servoRange = 0.4;
        boolean servoPos=false, servoPos2 = false;
//        basketArmServo.setPosition(0.0);
        basketActuationServo.setPosition(0.0);
        basketArmServo.setPosition(0.4);
        //Aiden - during competition day robot disconnected so we are trying this code
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (!isStopRequested()) {
            diSTANCe+=gamepad1.left_stick_y/100;
//            robot.TurretAngleControlRotating(diSTANCe);
            sleep(100);
            telemetry.addData("diSTANCe",diSTANCe);
            telemetry.addData("runtime",getRuntime());
            telemetry.update();
            if(gamepad1.a){
                if(servoPos2) {
                    basketActuationServo.setPosition(0.5);
                    telemetry.addData("here1", servoPos2);
                }
                else{
                    basketActuationServo.setPosition(1.0);
                    telemetry.addData("here2", servoPos2);
                }
                servoPos2 = !servoPos2;

            }
            if(gamepad1.b){
                if(servoPos) {
                    basketArmServo.setPosition(0.00);
                }
                else{
                    basketArmServo.setPosition(0.45);
                }
                servoPos = !servoPos;
            }
        }

        idle();
    }
}