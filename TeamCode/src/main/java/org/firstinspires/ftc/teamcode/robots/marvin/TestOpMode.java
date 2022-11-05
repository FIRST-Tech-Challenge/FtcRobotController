package org.firstinspires.ftc.teamcode.robots.marvin;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Iron Core OpMode", group="Challenge")
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
    // motor power
    private int armPosition = 0;
    private int elbowPositon = 0;
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
        wrist.setPosition(0);
    }
    @Override
    public void loop() {
        //tankDrive();
        mechanumDrive();
        armMove();
        clawMove();
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
    public void mechanumDrive()
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
        telemetry.addData("arm position: ", arm.getCurrentPosition());
        telemetry.addData("elbow position: ", elbow.getCurrentPosition());
        if(gamepad1.dpad_down)
        {
            if(currArmPos < 0 )
                currArmPos += 10;
            else
                currArmPos = 0;
        }//-1898, -360, -2073, -2582
        if(gamepad1.dpad_up)
        {
            if (currArmPos > -1890)
                currArmPos -= 10;
            else
                currArmPos = -1890;
        }
        if(gamepad1.dpad_right)
        {
            elbow.setTargetPosition(elbowPositon += 10);
        }
        if (gamepad1.dpad_left)
        {
//            if (elbowPositon > 0)
            elbow.setTargetPosition(elbowPositon -= 10);
        }
        if (gamepad1.left_trigger > DEADZONE)
        {
            wrist.setPosition(1);
        }
        if (gamepad1.right_trigger > DEADZONE)
        {
            wrist.setPosition(0);
        }
        arm.setTargetPosition(currArmPos);
    }
    public void clawMove() {
        telemetry.addData("Claw servo position:", claw.getPosition());
        if (gamepad1.left_bumper)
            claw.setPosition(.5);
        if (gamepad1.right_bumper)
            claw.setPosition(0);
    }

}
