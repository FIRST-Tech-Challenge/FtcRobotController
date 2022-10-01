package org.firstinspires.ftc.teamcode.robots.idk;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Basic: OpMode", group="idkBot")
public class TestOpMode extends OpMode {
    //variable setup
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackRight = null;
    // regular drive
    private double powerLeft = 0;
    private double powerRight = 0;
    // mecanum types
    private double powerFrontLeft = 0;
    private double powerFrontRight = 0;
    private double powerBackLeft = 0;
    private double powerBackRight = 0;
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
    }
    @Override
    public void loop() {
        //tankDrive();
        mechanumDrive();
    }
    public void tankDrive()
    {
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
}
