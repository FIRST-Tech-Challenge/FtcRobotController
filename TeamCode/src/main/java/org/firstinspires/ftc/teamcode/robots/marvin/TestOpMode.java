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
    private DcMotor elevator = null;
    private Servo claw = null;
    // regular drive
    private double powerLeft = 0;
    private double powerRight = 0;
    // motor power
    private double powerElevator = 0;
    private double powerClaw = 0;
    //number variables
    private static final float DEADZONE = .1f;
    private static final int MAXELEVHEIGHT = Integer.MAX_VALUE;
    private static final int MINELEVHEIGHT = 0;
    private int currElevHeight = 0;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing " + this.getClass()+"...");
        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();
        motorFrontLeft = this.hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = this.hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = this.hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = this.hardwareMap.get(DcMotor.class, "motorBackRight");
//        elevator = this.hardwareMap.get(DcMotor.class, "elevator");
//        claw = this.hardwareMap.get(Servo.class, "claw");
        this.motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        this.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {
        //tankDrive();
        mechanumDrive();
//        elevatorMove();
//        clawMove();
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
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        motorFrontLeft.setPower(v1);
        motorFrontRight.setPower(v2);
        motorBackLeft.setPower(v3);
        motorBackRight.setPower(v4);
    }
//    public void elevatorMove()
//    {
//        telemetry.addData("Elevator Position: ", elevator.getCurrentPosition());
//        powerElevator = 0;
//        if(gamepad1.right_trigger > DEADZONE)
//        {
//            powerElevator = gamepad1.right_trigger;
//        }
//        if(gamepad1.left_trigger > DEADZONE)
//        {
//            powerElevator = -gamepad1.left_trigger;
//        }
//        elevator.setPower(powerElevator);
//        currElevHeight += powerElevator;
//    }
//    public void clawMove() {
//        telemetry.addData("Claw servo position:", claw.getPosition());
//        if (gamepad1.left_bumper)
//            claw.setPosition(.5);
//        if (gamepad1.right_bumper)
//            claw.setPosition(0);
//    }
}
