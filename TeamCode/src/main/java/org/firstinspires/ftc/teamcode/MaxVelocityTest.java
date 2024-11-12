package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name="MaxVelocity Test", group = "TeleOp")
//@Disabled
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;
    double minVelocity = 99999.0;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "motor_fl");
        initMotor(motor);
        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(1);
            currentVelocity = motor.getVelocity();
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            } else if (currentVelocity < minVelocity) {
                minVelocity = currentVelocity;
            }
            telemetry.addData("current power", motor.getPower());
            Log.i("BAB", "current power : "+ motor.getPower());
            telemetry.addData("current velocity", motor.getVelocity());
            Log.i("BAB", "current velocity : "+ currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            Log.i("BAB", "maximum velocity : "+ maxVelocity);
            telemetry.addData("Minimum velocity", minVelocity);
            Log.i("BAB", "minimum velocity : "+ minVelocity);
            telemetry.update();
        }
    }

    public void initMotor(DcMotorEx driveMotor){
        driveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        driveMotor.setPower(0.0);
        driveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        driveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}