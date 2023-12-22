
package org.firstinspires.ftc.teamcode.blue.right;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AUTOBLleft_Blue_Right {

    private Servo leftclaw;
    private Servo rightclaw;
    private Servo flip;
    private Servo turnleft;
    private Servo turnright;
    private DcMotor upleft;
    private DcMotor hangupleft;
    private DcMotor rightmotor1;
    private DcMotor rightmotor2;
    private DcMotor upright;
    private DcMotor leftmotor1;
    private DcMotor leftmotor2;


    protected HardwareMap hardwareMap = null;
    protected Telemetry telemetry;

    AUTO22715_Blue_Right mainLoopClass;

    private void initBeforeOpMode(){
        leftclaw = hardwareMap.get(Servo.class, "left claw");
        rightclaw = hardwareMap.get(Servo.class, "right claw");
        flip = hardwareMap.get(Servo.class, "flip");
        turnleft = hardwareMap.get(Servo.class, "turn left");
        turnright = hardwareMap.get(Servo.class, "turn right");
        upleft = hardwareMap.get(DcMotor.class, "up left");
        hangupleft = hardwareMap.get(DcMotor.class, "hang up left");
        rightmotor1 = hardwareMap.get(DcMotor.class, "right motor 1");
        rightmotor2 = hardwareMap.get(DcMotor.class, "right motor 2");
        upright = hardwareMap.get(DcMotor.class, "up right");
        leftmotor1 = hardwareMap.get(DcMotor.class, "left motor 1");
        leftmotor2 = hardwareMap.get(DcMotor.class, "left motor 2");

        // Put initialization blocks here.
        leftclaw.setDirection(Servo.Direction.REVERSE);
        rightclaw.setDirection(Servo.Direction.REVERSE);
        flip.setDirection(Servo.Direction.REVERSE);
        flip.setPosition(0.06);
        leftclaw.setPosition(0.6);
        rightclaw.setPosition(0.6);
        turnleft.setDirection(Servo.Direction.REVERSE);
        turnleft.setPosition(0.05);
        turnright.setPosition(0.05);
        upleft.setDirection(DcMotorSimple.Direction.REVERSE);
        hangupleft.setDirection(DcMotorSimple.Direction.REVERSE);
        rightmotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightmotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init(AUTO22715_Blue_Right main){
        mainLoopClass = main;
        hardwareMap = mainLoopClass.hardwareMap;
        telemetry = mainLoopClass.telemetry;

        initBeforeOpMode();
    }

    private void waitForStart(){
        mainLoopClass.waitForStart();
    }

    private void sleep(long milliseconds){
        mainLoopClass.sleep(milliseconds);
    }

    private boolean opModeIsActive() {
        return mainLoopClass.opModeIsActive();
    }

    private void idle() {
        mainLoopClass.idle();
    }

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    public void runOpMode() {
        waitForStart();
        forward_1();
        sleep(500);
        right_turn();
        sleep(500);
        black1();
        sleep(1000);
        rightclaw.setPosition(0.3);
        sleep(1000);
        black1();
        rightclaw.setPosition(0.6);
        sleep(1000);
        Right_translation2();
        black2();
        sleep(500);
        turnleft.setPosition(1);
        turnright.setPosition(1);
        sleep(1000);
        upleft.setPower(-0.65);
        upright.setPower(0.65);
        sleep(1000);
        flip.setPosition(1);
        sleep(1000);
        leftclaw.setPosition(1);
        rightclaw.setPosition(0.3);
        sleep(1000);
        flip.setPosition(0.1);
        sleep(1000);
        turnleft.setPosition(0.05);
        turnright.setPosition(0.05);
        sleep(1000);
        leftclaw.setPosition(0.6);
        rightclaw.setPosition(0.6);
        sleep(1000);
        upleft.setPower(0.2);
        upright.setPower(-0.2);
        Right_translation();
    }

    /**
     * Describe this function...
     */
    private void forward_encoder(int left1Target, int left2Target, int right1Targt, int right2Target, double speed) {
        leftmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftmotor1.setTargetPosition(left1Target);
        leftmotor2.setTargetPosition(left2Target);
        rightmotor1.setTargetPosition(right1Targt);
        rightmotor2.setTargetPosition(right2Target);
        leftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftmotor1.setPower(speed);
        leftmotor2.setPower(speed);
        rightmotor1.setPower(speed);
        rightmotor2.setPower(speed);
        while (opModeIsActive() && (leftmotor1.isBusy() && leftmotor2.isBusy()) == (rightmotor1.isBusy() && rightmotor2.isBusy())) {
            idle();
        }
    }

    /**
     * Describe this function...
     */
    private void forward_1() {
        forward_encoder((1000 / 53) * 75, (1000 / 53) * 75, (1000 / 53) * 75, (1000 / 53) * 75, 0.5);
    }

    /**
     * Describe this function...
     */
    private void black1() {
        forward_encoder(-((1000 / 53) * 40), -((1000 / 53) * 40), -((1000 / 53) * 40), -((1000 / 53) * 40), 0.5);
    }

    /**
     * Describe this function...
     */
    private void right_turn() {
        forward_encoder((1000 / 53) * 46, (1000 / 53) * 46, -((1000 / 53) * 46), -((1000 / 53) * 46), 0.5);
    }

    /**
     * Describe this function...
     */
    private void forward_2() {
        forward_encoder((1000 / 53) * 20, (1000 / 53) * 20, (1000 / 53) * 20, (1000 / 53) * 20, 0.5);
    }

    /**
     * Describe this function...
     */
    private void black2() {
        forward_encoder(-((1000 / 53) * 12), -((1000 / 53) * 12), -((1000 / 53) * 12), -((1000 / 53) * 12), 0.5);
    }

    /**
     * Describe this function...
     */
    private void Right_translation() {
        forward_encoder((1000 / 53) * 60, -((1000 / 53) * 60), -((1000 / 53) * 60), (1000 / 53) * 60, 0.5);
    }

    /**
     * Describe this function...
     */
    private void Right_translation2() {
        forward_encoder((1000 / 53) * 38, -((1000 / 53) * 38), -((1000 / 53) * 38), (1000 / 53) * 38, 0.5);
    }
}