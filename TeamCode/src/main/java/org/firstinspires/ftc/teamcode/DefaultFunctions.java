package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DefaultFunctions {
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor armMotor1;
    private DcMotor armMotor2;
    private Servo intakeServo;
    private LinearOpMode opmode;

    int currentHeight = 0;


    public void initAutonomous(HardwareMap hardwareMap, LinearOpMode opmode) {
        this.opmode = opmode;

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        armMotor1 = hardwareMap.get(DcMotor.class, "armmotor1");
        armMotor2= hardwareMap.get(DcMotor.class, "armmotor2");

        intakeServo = hardwareMap.get(Servo.class, "Intake");

        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void drive(int leftBackTicks, int rightBackTicks, int leftFrontTicks, int rightFrontTicks) {
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        leftBack.setTargetPosition(leftBackTicks);
        rightBack.setTargetPosition(rightBackTicks);
        leftFront.setTargetPosition(leftFrontTicks);
        rightFront.setTargetPosition(rightFrontTicks);

        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(leftBack.isBusy() || rightBack.isBusy() || leftFront.isBusy() || rightFront.isBusy()) {
            opmode.idle();
        }
    }
    public void driveLeft(int Ticks) {
        drive(Ticks, Ticks, -Ticks, -Ticks);
    }
    public void driveRight(int Ticks) {
        drive(-Ticks, -Ticks, Ticks, Ticks);
    }
    public void driveForward(int Ticks) {
        drive(Ticks, -Ticks, Ticks, -Ticks);
    }
    public void driveBackward(int Ticks) {
        drive(-Ticks, Ticks, -Ticks, Ticks);
    }
    public void turnLeft (int degrees) {int Ticks = degrees * 70 / 6; drive(-Ticks, -Ticks, -Ticks, -Ticks); }
    public void turnRight (int degrees) {int Ticks = degrees * 70 / 6; drive (Ticks, Ticks, Ticks, Ticks); }
    public void armMovement(int armTicks) {
        armMotor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        armMotor1.setTargetPosition(armTicks);
        armMotor1.setPower(0.5);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        armMotor2.setTargetPosition(armTicks);
        armMotor2.setPower(0.5);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armMotor1.isBusy()) {
            opmode.idle();
        }
    }

    /** input ibtergers beschrijving. Output: real
     * beschrijf kort en krachtig de werking van de functoe
     * @param servoPosition
     */
    public void servoPositioning(double servoPosition) {intakeServo.setPosition(servoPosition);} /*pos 0.3 = closed, pos 0.5 = open*/
    public int armGet (int stackNum) {
        int[][] stack = { {0, 400}, {400, 350}, {420, 320}, {525, 300}, {625, 280} };
        servoPositioning(0.5);
        armMovement(stack[stackNum][0] - currentHeight);
        driveForward(stack[stackNum][1]);
        servoPositioning(0.3);
        armMovement(400);
        driveBackward(stack[stackNum][1]);
        return armMotor1.getCurrentPosition();
    }
    public int armDrop (int junctionNum) {
        int[] junction = {150, 800, 1300, 1800};
        armMovement(junction[junctionNum] - currentHeight);
//        armMovement(-400);
        servoPositioning(0.5);
        return armMotor1.getCurrentPosition();
    }
}