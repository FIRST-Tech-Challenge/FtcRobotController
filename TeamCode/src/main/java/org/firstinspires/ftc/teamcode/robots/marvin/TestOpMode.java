package org.firstinspires.ftc.teamcode.robots.marvin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Iron Core OpMode", group="Challenge")
public class TestOpMode extends OpMode {
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
    //booleans
    private boolean auton = true;
    private boolean
    // motor power
    private int elbowPosition = 0;
    private int targetElbowPosition = 0;
    private double wristPosition = 0;
    private double targetWristPosition = 0;
    // arm and claw variables
    private int armPosition = 0;
    private int targetArmPos = 0;
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
        elbowPosition = elbow.getCurrentPosition();
        wristPosition = wrist.getPosition();
        wrist.setPosition(0);
    }
    @Override
    public void loop() {
        if (auton){


        }

        updateSensors();
        //tankDrive();
        //process drive inputs
        mecanumDrive();
        //joystick processing
        presets();
        armMove();
        clawMove();
        updateMotors();
    }

    public void updateSensors(){
        //get current positions
        armPosition=arm.getCurrentPosition();
        elbowPosition=elbow.getCurrentPosition();
        wristPosition = wrist.getPosition();

    }
    public void updateMotors(){
        arm.setTargetPosition(targetArmPos);
        elbow.setTargetPosition(targetElbowPosition);
        wrist.setPosition(targetWristPosition);
    }

    public void presets() {
        if (gamepad1.a) { //pickup cone
            targetArmPos = -378;
            targetElbowPosition = -591;
            targetWristPosition = 1;
        }
        if (gamepad1.b) { // low junction
            targetArmPos = -771;
            targetElbowPosition = -290;
            targetWristPosition = 0.5;
        }

        if (gamepad1.x) { //medium junction
            targetArmPos = -1440;
            targetElbowPosition = -820;
            targetWristPosition = 0.5;
        }
        if (gamepad1.y) { //high junction
            targetArmPos = -1840;
            targetElbowPosition =  -1050;
            targetWristPosition = 0.85;
        }
    }

    public void tankDrive()
    {
        powerRight = 0;
        powerLeft = 0;

// tanvi is the bestestestestestest

        if(Math.abs(gamepad1.left_stick_y) > DEADZONE)
        {
            powerLeft = gamepad1.left_stick_y;
        }
        if(Math.abs(gamepad1.right_stick_y) > DEADZONE)
        {
            powerRight = gamepad1.right_stick_y;
        }
        motorFrontRight.setPower(powerRight);
        motorFrontLeft.setPower(powerLeft);
        motorBackRight.setPower(powerRight);
        motorBackLeft.setPower(powerLeft);
    }
    public void mecanumDrive()
    {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;
        motorFrontLeft.setPower(v1);
        motorFrontRight.setPower(v2);
        motorBackLeft.setPower(v3);
        motorBackRight.setPower(v4);
    }
    public void armMove()
    {
        telemetry.addData("arm position: ", armPosition);
        telemetry.addData("elbow position: ", elbowPosition);
        telemetry.addData("wrist position: ", wristPosition);
        if(gamepad1.dpad_down) //manually lower arm - shoulder joint
        {
            targetArmPos = armPosition + 30;
            if(targetArmPos > 0 )
                targetArmPos = 0;
        }
        if(gamepad1.dpad_up) //manually raise arm - shoulder joint
        {
            targetArmPos = armPosition - 30;
            if (targetArmPos < -1890)
                targetArmPos = -1890;
        }
        if(gamepad1.dpad_right) //contract elbow
        {
            targetElbowPosition = elbowPosition + 30;
            if(targetElbowPosition > 0 )
                targetElbowPosition = 0;

        }
        if (gamepad1.dpad_left) //extend elbow
        {
            targetElbowPosition = elbowPosition - 30;
            if (targetElbowPosition < -1100) //todo might want to allow a little more manual extension? currently limited to the high junction elbow position
                targetElbowPosition = -1100;

        }
        if (gamepad1.left_trigger > DEADZONE)
        {
            wrist.setPosition(wrist.getPosition()+.02);
        }
        if (gamepad1.right_trigger > DEADZONE)
        {
            wrist.setPosition(wrist.getPosition()-.02);
        }

    }
    public void clawMove() {
        telemetry.addData("Claw servo position:", claw.getPosition());
        if (gamepad1.left_bumper)
            claw.setPosition(claw.getPosition()+.02);
        if (gamepad1.right_bumper)
            claw.setPosition(claw.getPosition()-.02);
    }

}
