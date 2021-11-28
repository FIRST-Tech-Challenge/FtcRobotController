package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "TickTest", group = "TeleOp")
public class TickTest extends MasterOpMode{
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorDuck;
    DcMotor motorArm;
    Servo servoGrabber;
    Servo servoArm;
    @Override
    public void runOpMode() {
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorArm = hardwareMap.dcMotor.get("motorArm");
        motorDuck = hardwareMap.dcMotor.get("motorDuck");
        servoGrabber = hardwareMap.servo.get("servoGrabber");
        servoArm = hardwareMap.servo.get("servoArm");
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDuck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set run mode of arm motor (encoders --> run to position)
        motorArm.setTargetPosition(0);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoGrabber.setPosition(0.34);
        pauseMillis(500);
        servoArm.setPosition(0.01);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                Forward(72, 0.5);
            }
            else if (gamepad1.dpad_down) {
                TurnAngle(90);
            }
        }
    }
}