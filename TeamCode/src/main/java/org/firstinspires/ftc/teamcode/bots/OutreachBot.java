package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OutreachBot {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    protected HardwareMap hwMap = null;
    protected Telemetry telemetry;
    protected LinearOpMode owner = null;

    private ElapsedTime runtime = new ElapsedTime();

    public static String LEFT_FRONT = "frontLeft";
    public static String RIGHT_FRONT = "frontRight";
    public static String LEFT_BACK = "backLeft";
    public static String RIGHT_BACK = "backRight";

    public OutreachBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode owner, HardwareMap hw, Telemetry t) throws Exception {
        // Save reference to Hardware map
        this.owner = owner;
        this.hwMap = hw;
        this.telemetry = t;

        try {
            // Define and Initialize Motors
            frontLeft = hwMap.get(DcMotor.class, LEFT_FRONT);
            frontRight = hwMap.get(DcMotor.class, RIGHT_FRONT);
            backLeft = hwMap.get(DcMotor.class, LEFT_BACK);
            backRight = hwMap.get(DcMotor.class, RIGHT_BACK);

            telemetry.addData("Init", "Drive");

            if (backLeft != null) {
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (backRight != null) {
                backRight.setDirection(DcMotor.Direction.FORWARD);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (frontLeft != null) {
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (frontRight != null) {
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        } catch (Exception ex) {
            //issues accessing drive resources
            throw new Exception("Issues accessing drive resources. Check the controller config", ex);
        }

        this.stop();
    }


    public void stop() {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {

            // Set all motors to zero power
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    public void move(double drive, double turn) {
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null) {
            double rightPower = Range.clip(drive + turn, -1.0, 1.0);
            double leftPower = Range.clip(drive - turn, -1.0, 1.0);

            //create dead zone for bad joysticks
            if (drive > 0) {
                if (Math.abs(rightPower) < 0.02) {
                    rightPower = 0;
                }

                if (Math.abs(leftPower) < 0.02) {
                    leftPower = 0;
                }
            }

            //apply logarithmic adjustment
            rightPower = rightPower * 100 / 110;
            rightPower = rightPower * rightPower * rightPower;

            leftPower = leftPower * 100 / 110;
            leftPower = leftPower * leftPower * leftPower;

            this.frontLeft.setPower(leftPower);
            this.frontRight.setPower(rightPower);
            this.backLeft.setPower(leftPower);
            this.backRight.setPower(rightPower);
        }
    }


    public void strafeRight(double speed) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            power = power * power * power;
            this.backLeft.setPower(-power);
            this.backRight.setPower(power);
            this.frontLeft.setPower(power);
            this.frontRight.setPower(-power);
            telemetry.addData("Motors", "Front: %.0f", power);
            telemetry.addData("Motors", "Back: %.0f", power);
        }
    }

    public void strafeLeft(double speed) {
        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
            double power = Range.clip(speed, -1.0, 1.0);
            power = power * power * power;
            this.backLeft.setPower(power);
            this.backRight.setPower(-power);
            this.frontLeft.setPower(-power);
            this.frontRight.setPower(power);
            telemetry.addData("Motors", "Front: %.0f", power);
            telemetry.addData("Motors", "Back: %.0f", power);
        }
    }


}
