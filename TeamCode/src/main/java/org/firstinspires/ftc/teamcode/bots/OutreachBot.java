package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OutreachBot {
    //  define motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;

    protected HardwareMap hwMap = null;
    protected Telemetry telemetry;
    protected LinearOpMode owner = null;

    private ElapsedTime runtime = new ElapsedTime();

    public OutreachBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode owner, HardwareMap hw, Telemetry t) throws Exception {
        // Save reference to Hardware map
        this.owner = owner;
        this.hwMap = hw;
        this.telemetry = t;

        frontLeft = hwMap.get(DcMotor.class, "left_front");
        frontRight = hwMap.get(DcMotor.class, "right_front");
        backLeft = hwMap.get(DcMotor.class, "left_back");
        backRight = hwMap.get(DcMotor.class, "right_back");

        try {
            // Define and Initialize Motors
//            frontLeft = hwMap.get(DcMotor.class, LEFT_FRONT);

            telemetry.addData("Init", "Drive");

            if (backLeft != null) {
                backLeft.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (backRight != null) {
                backRight.setDirection(DcMotor.Direction.REVERSE);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }  if (frontRight != null) {
                frontRight.setDirection(DcMotor.Direction.REVERSE);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }  if (frontLeft != null) {
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

        } catch (Exception ex) {
            //issues accessing drive resources
            throw new Exception("Issues accessing drive resources. Check the controller config", ex);
        }

        this.stop();
    }


    public void stop() {
        if(backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);
        }
    }

    public void move(double drive) {
        if(backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
            frontLeft.setPower(drive);
            frontRight.setPower(drive);
            backRight.setPower(drive);
            backLeft.setPower(drive);
        }
    }


    public void strafeRight(double speed) {
//        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
//            double power = Range.clip(speed, -1.0, 1.0);
//            power = power * power * power;
//            this.backLeft.setPower(-power);
//            this.backRight.setPower(power);
//            this.frontLeft.setPower(power);
//            this.frontRight.setPower(-power);
//            telemetry.addData("Motors", "Front: %.0f", power);
//            telemetry.addData("Motors", "Back: %.0f", power);
//        }
    }

    public void strafeLeft(double speed) {
//        if (backLeft != null && backRight != null && frontLeft != null && frontRight != null) {
//            double power = Range.clip(speed, -1.0, 1.0);
//            power = power * power * power;
//            this.backLeft.setPower(power);
//            this.backRight.setPower(-power);
//            this.frontLeft.setPower(-power);
//            this.frontRight.setPower(power);
//            telemetry.addData("Motors", "Front: %.0f", power);
//            telemetry.addData("Motors", "Back: %.0f", power);
//        }
    }


}
