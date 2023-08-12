package org.firstinspires.ftc.teamcode.robots.r2v2;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import android.view.animation.ScaleAnimation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.taubot.vision.pipeline.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config ("R2V2")
@TeleOp(name="R2V2", group="Challenge")
public class R2V2 extends OpMode {

    //motors
    DcMotorEx counterBrake, steering, accelerator;

    //variables
    long currentSteeringTicks;
    long waitTime;
    boolean calibrateFailed = false;
    public static int maxSteeringTicksPerSecond = 500;
    public int maxRightSteeringTicks, maxLeftSteeringTicks, centerSteeringTicks;
    int calibrateIndex = 0;

    FtcDashboard dashboard;
    public static double COUNTERBRAKE_TENSION_AMPS = 3.5;
    public static int COUNTERBRAKE_CALIBRATE_VELOCITY = 150;

    public int counterBrakeMaxTicks;
    public static double STEERING_STALL_AMPS = 4.5;
    boolean calibrated = false;

    @Override
    public void init() {
        boolean initialized = initMotors();
        dashboard = FtcDashboard.getInstance();
        telemetry.addData("Initializing: ", initialized);
        telemetry.update();
        calibrateIndex = 0;
        if(!initialized)
            stop();
    }

    @Override
    public void init_loop()
    {
        calibrateTelemetry();
        if(!calibrated)
        {
            calibrated = calibrate();
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry();
        if(calibrated) {
            //default actions
            steering.setPower(1);
            steering.setTargetPosition((int)(steering.getCurrentPosition() + (gamepad1.right_stick_x * maxSteeringTicksPerSecond)));

            //if left trigger (counterBrake) is intentionally depressed, allow acceleration and apply counterBrake
            if (gamepad1.left_trigger > .5) {
                counterBrake.setPower(1);
                counterBrake.setTargetPosition((int)(gamepad1.right_trigger * counterBrakeMaxTicks));
                //accelerator.setPower(gamepad1.right_trigger);
            }
            else
                counterBrake.setPower(0);
        }
        else {
            steering.setMotorDisable();
            counterBrake.setMotorDisable();
            accelerator.setMotorDisable();
        }
        telemetry.update();
    }
    @Override
    public void stop() {
        counterBrake.setPower(0); //removes counterBrake (holds brake)
        steering.setTargetPosition(steering.getCurrentPosition()); //locks steering
    }

    public void telemetry() {
        telemetry.addData("Steering Location:\t", steering.getCurrentPosition());
        telemetry.addData("Steering Amps:\t", steering.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("counterBrake Location:\t", counterBrake.getCurrentPosition());
        telemetry.addData("Calibrated:\t", calibrated);

    }

    public void calibrateTelemetry()
    {
        if(calibrateFailed) {
            telemetry.addData("CALIBRATION FAILED - CHECK HARDWARE", calibrateFailed);
        }
            else {
            telemetry.addData("Steering Location:\t", steering.getCurrentPosition());
            telemetry.addData("Steering Amps:\t", steering.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("CounterBreak:\t", counterBrake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Steering Max Left:\t", maxLeftSteeringTicks);
            telemetry.addData("Steering Max Right:\t", maxRightSteeringTicks);
            telemetry.addData("Steering Center:\t", centerSteeringTicks);
            telemetry.addData("Calibration Index", calibrateIndex);
            telemetry.addData("waitTime", waitTime);
            telemetry.addData("CounterBrake Position", counterBrake.getCurrentPosition());
            telemetry.addData("CounterBrake Max Ticks", waitTime);

        }
    }
    public boolean calibrate()
    {
        switch(calibrateIndex)
        {
            case 0: {
                steering.setPower(-1);
                if(steering.getCurrent(CurrentUnit.AMPS) > STEERING_STALL_AMPS || gamepad1.a) {
                    maxLeftSteeringTicks = steering.getCurrentPosition(); //store leftmost ticks (should be 0)
                    waitTime = futureTime(2);
                    calibrateIndex++;
                }
                break;
            }

            case 1: {
                steering.setPower(0);
                if(System.nanoTime() > waitTime) {
                    calibrateIndex++;
                }
                break;
            }


            case 2: {
                steering.setPower(1);
                if(steering.getCurrent(CurrentUnit.AMPS) > STEERING_STALL_AMPS || gamepad1.a) {
                    maxRightSteeringTicks = steering.getCurrentPosition(); //finds the ticks at the rightmost position of the wheel based on stall
                    waitTime = futureTime(2);
                    calibrateIndex++;
                }
                break;
            }

            case 3: {
                steering.setPower(0);
                if(System.nanoTime() > waitTime){
                    calibrateIndex ++;
                }
                break;
            }

            case 4: {
                steering.setPower(1);
                centerSteeringTicks = maxLeftSteeringTicks + ((Math.abs(maxLeftSteeringTicks) + Math.abs(maxRightSteeringTicks)) / 2);
                steering.setTargetPosition(centerSteeringTicks);
                steering.setMode(DcMotor.RunMode.RUN_TO_POSITION); //switch back to encoders to go to center
                //assume center is halfway between left max and right max
                //go to center and set center to 0, making left negative and right positive
                if(steering.getCurrentPosition() == steering.getTargetPosition()) {
                    steering.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    steering.setTargetPosition(0);
                    steering.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    maxLeftSteeringTicks += centerSteeringTicks;
                    maxRightSteeringTicks += centerSteeringTicks;
                    centerSteeringTicks = 0;
                    calibrateIndex++;
                }
                break;
            }
            case 5: {
                if(maxRightSteeringTicks != 0 && maxLeftSteeringTicks != 0) {
                    counterBrake.setPower(1);
                    calibrateIndex++;
                }
                else {
                    calibrateFailed = true;
                }

                break;
            }
            case 6: {
                if (counterBrake.getCurrent(CurrentUnit.AMPS) > COUNTERBRAKE_TENSION_AMPS || gamepad1.x) {
                    counterBrake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    counterBrake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    counterBrake.setVelocity(COUNTERBRAKE_CALIBRATE_VELOCITY);
                    calibrateIndex++;
            }
                break;
            }

            case 7: {
                if (gamepad1.b) {
                    counterBrakeMaxTicks = counterBrake.getCurrentPosition();
                    counterBrake.setTargetPosition(0);
                    counterBrake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    calibrateIndex++;
                }
                break;
            }
            case 8: {
                calibrated = true;
                return true;
            }
        }
        return false;
    }

    public boolean initMotors() {
        try {
            counterBrake = this.hardwareMap.get(DcMotorEx.class, "brake");
            steering = this.hardwareMap.get(DcMotorEx.class, "steering");
            accelerator = this.hardwareMap.get(DcMotorEx.class, "accelerator");
            counterBrake.setMotorEnable();
            accelerator.setMotorEnable();
            steering.setMotorEnable();
            counterBrake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            steering.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            accelerator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            counterBrake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            steering.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            accelerator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return true;
        }
        catch (Exception e) {
            telemetry.addData("uh oh information: ", e.getStackTrace());
            return false;
        }
    }
}
