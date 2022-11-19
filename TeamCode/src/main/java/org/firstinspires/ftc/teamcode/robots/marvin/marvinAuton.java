package org.firstinspires.ftc.teamcode.robots.marvin;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "marvinAuton")
public class marvinAuton extends OpMode {
    //variable setup
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor arm = null;
    private DcMotor elbow = null;
    private Servo claw = null;
    private Servo wrist = null;
    // regular drive
    private double powerLeft = 0;
    private double powerRight = 0;
    // motor power
    private int armPosition = 0;
    private int elbowPositon = 0;
    private double wristPosition = 0;
    // arm and claw variables
    private int currArmPos = 0;
    private int maxArm = Integer.MAX_VALUE;
    //number variables
    private static final float DEADZONE = .1f;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing " + this.getClass()+"...");
        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();
        motorFrontLeft = this.hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = this.hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = this.hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = this.hardwareMap.get(DcMotor.class, "motorBackRight");
        arm = this.hardwareMap.get(DcMotor.class, "arm");
        elbow = this.hardwareMap.get(DcMotor.class, "elbow");
        claw = this.hardwareMap.get(Servo.class, "claw");
        wrist = this.hardwareMap.get(Servo.class, "wrist");
        this.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        this.motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        elbow.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
        elbow.setPower(1);
//        elbow.setTargetPosition(-505);
        elbowPositon = elbow.getCurrentPosition();
        wristPosition = wrist.getPosition();
        wrist.setPosition(0);

    }

    @Override
    public void loop() {
        telemetry.addData("autonomous running...", String.valueOf(motorFrontLeft.getCurrentPosition()), motorFrontRight.getCurrentPosition());
        driveForward();
    }

    public void driveForward (){
        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + 100);
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + 100);
        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + 100);
        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + 100    );

    }
}
