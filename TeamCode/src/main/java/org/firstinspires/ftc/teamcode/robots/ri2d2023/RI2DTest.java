package org.firstinspires.ftc.teamcode.robots.ri2d2023;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="RI2D Test", group="Challenge")
public class RI2DTest extends OpMode {
    //variable setup
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackRight = null;
    public static float DEADZONE = .1f;


    @Override
    public void init() {
        telemetry.addData("Status", "Initializing " + this.getClass() + "...");
        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();
        motorFrontLeft = this.hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = this.hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = this.hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = this.hardwareMap.get(DcMotor.class, "motorBackRight");
        this.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        this.motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.a)
            motorBackRight.setPower(0.5);
        else if (gamepad1.y)
            motorFrontRight.setPower(0.5);
        else if (gamepad1.dpad_up)
            motorFrontLeft.setPower(0.5);
        else if(gamepad1.dpad_down)
            motorBackLeft.setPower(0.5);
        else {
            motorFrontLeft.setPower(0);
            motorFrontLeft.setPower(0);
            motorFrontLeft.setPower(0);
            motorFrontLeft.setPower(0);
        }
    }

}

