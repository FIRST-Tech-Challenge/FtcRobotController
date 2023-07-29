package org.firstinspires.ftc.teamcode.robots.r2v2;

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
    public static int maxSteeringTicksPerSecond = 100, maxRightSteeringTicks, maxLeftSteeringTicks, centerSteeringTicks;
    int calibrateIndex = 0;
    public static double STEERING_STALL_AMPS = 2;
    boolean calibrated = false;

    @Override
    public void init() {
        boolean initialized = initMotors();
        telemetry.addData("Initializing: ", initialized);
        telemetry.update();
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
    }

    @Override
    public void loop() {
        telemetry();
        //default actions
        counterBrake.setPower(0);
        steering.setVelocity(gamepad1.right_stick_x*maxSteeringTicksPerSecond);

        //if left trigger (counterBrake) is intentionally depressed, allow acceleration and apply counterBrake
        if(gamepad1.left_trigger > .5) {
            counterBrake.setPower(gamepad1.right_trigger);
            //accelerator.setPower(gamepad1.right_trigger);
        }
    }
    @Override
    public void stop() {
        counterBrake.setPower(0); //removes counterBrake (holds brake)
        steering.setTargetPosition(steering.getCurrentPosition()); //locks steering
    }

    public void telemetry() {
        telemetry.addData("Steering Location:\t", steering.getCurrentPosition());
        telemetry.addData("Steering Amps:\t", steering.getCurrent(CurrentUnit.AMPS));
    }

    public void calibrateTelemetry()
    {
        telemetry.addData("Steering Location:\t", steering.getCurrentPosition());
        telemetry.addData("Steering Amps:\t", steering.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Steering Max Left:\t", maxLeftSteeringTicks);
        telemetry.addData("Steering Max Right:\t", maxRightSteeringTicks);
        telemetry.addData("Steering Center:\t", centerSteeringTicks);
    }
    public boolean calibrate()
    {
        switch(calibrateIndex)
        {
            case 0: {
                steering.setPower(-1);
                if(steering.getCurrent(CurrentUnit.AMPS) > STEERING_STALL_AMPS) {
                    steering.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//once left is found, use left as 0 ticks
                    maxLeftSteeringTicks = steering.getCurrentPosition(); //store leftmost ticks (should be 0)
                    calibrateIndex++;
                }
                break;
            }

            case 1: {
                steering.setPower(1);
                if(steering.getCurrent(CurrentUnit.AMPS) > STEERING_STALL_AMPS) {
                    maxRightSteeringTicks = steering.getCurrentPosition(); //finds the ticks at the rightmost position of the wheel based on stall
                    steering.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //switch back to encoders to find center in next stage
                    steering.setTargetPosition(steering.getCurrentPosition());
                    calibrateIndex++;
                }
                break;
            }
            case 2: {
                steering.setPower(1);
                steering.setTargetPosition(centerSteeringTicks = ((maxRightSteeringTicks - maxLeftSteeringTicks) / 2));
                //assume center is halfway between left max and right max
                //go to center and set center to 0, making left negative and right positive
                if(steering.getCurrentPosition() <= centerSteeringTicks) {
                    steering.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    maxLeftSteeringTicks = -centerSteeringTicks;
                    maxRightSteeringTicks = centerSteeringTicks;
                    centerSteeringTicks = 0;
                    calibrateIndex++;
                }
                break;
            }
            case 3: {
                if(true)
                    calibrateIndex++;
                break;
            }
            case 4: {
                if(true)
                    calibrateIndex++;
                break;
            }
            case 5: {
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
