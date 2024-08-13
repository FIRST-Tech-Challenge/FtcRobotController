package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputImpl;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Tank Tele-Op", group = "Robot")

public class OutreachBot2024 extends OpMode {

    /* Declare OpMode members. */
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public RevBlinkinLedDriver FriedFries = null;
    public RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
    public Servo triggerServo = null;
    public Servo piston = null;
    public PwmControl pistonController = null;
    double x;
    public void init() {
        // Define and Initialize Motor
        //Dear robot, why wont you work???,i need you rn, plz btfm,youre my little pookie ber, we care about youz,ur lovd!
        x=0;
        frontLeft = hardwareMap.get(DcMotor.class, "LF");
        backLeft = hardwareMap.get(DcMotor.class, "LB");
        frontRight = hardwareMap.get(DcMotor.class, "RF");
        backRight = hardwareMap.get(DcMotor.class, "RB");
        FriedFries = hardwareMap.get(RevBlinkinLedDriver.class, "snuffleupagusLED");
        triggerServo = hardwareMap.get(Servo.class, "TriggerS");
        piston = hardwareMap.get(Servo.class, "Piston");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pistonController = (PwmControl) piston;
        pistonController.setPwmRange(new PwmControl.PwmRange(0.75,2.27));



        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    public void init_loop() {
    }

    public void start() {
        telemetry.clear();
    }

    public void loop() {
        double left1y;
        double right1x;

        left1y = gamepad1.left_stick_y;
        right1x = gamepad1.right_stick_x;

        double leftPower = Math.min(Math.abs(left1y + right1x), 1) * Math.signum(left1y + right1x);
        double rightPower = Math.min(Math.abs(left1y - right1x), 1) * Math.signum(left1y - right1x);
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);
        if (gamepad1.a){
            pattern = pattern.next();
            if (pattern == RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE) {
                pattern = pattern.next();
            }
            FriedFries.setPattern(pattern);
            telemetry.addData("pattern: ", pattern.toString());
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if (gamepad1.b) {
            pattern = pattern.previous();
            telemetry.addData("pattern: ", pattern.toString());
            if (pattern == RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE) {
                pattern = pattern.previous();
            }
            FriedFries.setPattern(pattern);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            if (gamepad1.dpad_right) {
                piston.setPosition(0.5);
            }
            if (gamepad1.dpad_left) {
                piston.setPosition(0);

            }
            if (gamepad1.dpad_down) {
                triggerServo.setPosition(0);
            }
            if (gamepad1.dpad_up) {
                triggerServo.setPosition(0.75);
            }
            FriedFries.setPattern(pattern);
        }
    }
}
