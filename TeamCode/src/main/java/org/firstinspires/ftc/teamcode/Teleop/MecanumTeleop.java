package org.firstinspires.ftc.teamcode.Teleop;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.OdometryTracker;

@TeleOp(name = "MecanumTeleOp(Weakened)")
public class MecanumTeleop extends LinearOpMode {
    LinearOpMode op = this;
    DcMotorEx motorLeftFront;
    DcMotorEx motorRightFront;
    DcMotorEx motorLeftBack;
    DcMotorEx motorRightBack;

    public void runOpMode() {
        motorLeftFront = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightBack");
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        OdometryTracker tracker = new OdometryTracker(this,false,false);
        tracker.setPosition(0,0,0);
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        resetStartTime();
        while (!isStopRequested()&&getRuntime()<90) {
            float leftStickx = op.gamepad1.left_stick_x,leftSticky = -op.gamepad1.left_stick_y,
                    rightStick = op.gamepad1.right_stick_x*0.5f;
            double leftStickr = sqrt(pow(leftSticky, 2) + pow(leftStickx, 2))*0.5,powera,powerb,
                    angle = atan2(leftSticky,leftStickx);

            powera = -sin(angle + PI/4);
            powerb = -sin(angle - PI/4);
            motorLeftFront.setPower(powerb * leftStickr + rightStick);
            motorRightBack.setPower(powerb * leftStickr - rightStick);
            motorRightFront.setPower(powera * leftStickr - rightStick);
            motorLeftBack.setPower(powera * leftStickr + rightStick);
            tracker.track();
        }

        idle();
    }
}
