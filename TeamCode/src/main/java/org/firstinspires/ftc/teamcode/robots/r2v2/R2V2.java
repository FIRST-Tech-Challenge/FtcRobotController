package org.firstinspires.ftc.teamcode.robots.r2v2;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config("R2V2")
@TeleOp(name = "R2V2", group = "Challenge")
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
        telemetry = dashboard.getTelemetry();
        telemetry.update();
        calibrateIndex = 0;
        if (!initialized)
            stop();
    }

    @Override
    public void init_loop() {
        calibrateTelemetry();
        if (!calibrated) {
            calibrated = calibrate();
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry();
        if (calibrated) {
            //emergency stop button
            //locks steering and disables counterbrake and accelerator to be backdriven
            if (gamepad1.b || gamepad2.b) {
                steering.setMotorEnable();
                counterBrake.setMotorDisable();
                accelerator.setMotorDisable();
                stop();
            }
            //default actions
            steering.setPower(1);
            //add a fraction of the max steering tps depending on gamepad input
            //check if current gamepad input will send steering wheel out of maxLeft or maxRight
//            if (
//                    (steering.getCurrentPosition() + (gamepad1.right_stick_x * maxSteeringTicksPerSecond) > maxRightSteeringTicks)
//                ||  (steering.getCurrentPosition() + (gamepad1.right_stick_x * maxSteeringTicksPerSecond) < maxLeftSteeringTicks)
//            ) {
                steering.setTargetPosition((int) (steering.getCurrentPosition() + (-gamepad1.right_stick_x * maxSteeringTicksPerSecond)));
//            }
//            else {
//                steering.setTargetPosition(steering.getCurrentPosition());
//            }

            //if deadman's switch (on second gamepad) is intentionally depressed, allow acceleration and apply counterBrake
            if (gamepad2.left_trigger > .5) {
                counterBrake.setPower(1);
                counterBrake.setTargetPosition((int) (gamepad1.right_trigger * counterBrakeMaxTicks));
                //accelerator.setPower(gamepad1.right_trigger);
            } else {
                counterBrake.setPower(0);
            }
        } else {
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
        if(calibrated && !calibrateFailed) {
            telemetry.addData("Steering Location:\t", steering.getCurrentPosition());
            telemetry.addData("Steering Amperage:\t", steering.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("CounterBrake Location:\t", counterBrake.getCurrentPosition());
            telemetry.addData("CounterBrake Amperage:\t", counterBrake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("CounterBrake Max Ticks", counterBrakeMaxTicks);
            telemetry.addData("Steering Max Left:\t", maxLeftSteeringTicks);
            telemetry.addData("Steering Max Right:\t", maxRightSteeringTicks);
        }
        else {
            telemetry.addData("CALIBRATION NOT COMPLETE; ROBOT WILL NOT RESPOND", "");
        }


    }

    public void calibrateTelemetry() {
        if (calibrateFailed) {
            telemetry.addData("CALIBRATION FAILED - CHECK HARDWARE", "");
        } else {
            telemetry.addData("Steering Location:\t", steering.getCurrentPosition());
            telemetry.addData("Steering Amps:\t", steering.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("CounterBrake Amps:\t", counterBrake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Steering Max Left:\t", maxLeftSteeringTicks);
            telemetry.addData("Steering Max Right:\t", maxRightSteeringTicks);
            telemetry.addData("Steering Center:\t", centerSteeringTicks);
            telemetry.addData("Calibration Index", calibrateIndex);
            telemetry.addData("CounterBrake Position", counterBrake.getCurrentPosition());
            telemetry.addData("CounterBrake Max Ticks", counterBrakeMaxTicks);

        }
    }

    public boolean calibrate() {
        switch (calibrateIndex) {
            case 0: {
                steering.setPower(-1);
                if (steering.getCurrent(CurrentUnit.AMPS) > STEERING_STALL_AMPS || gamepad1.a) {
                    maxLeftSteeringTicks = steering.getCurrentPosition(); //store leftmost ticks based on stall (should be ~-11k)
                    waitTime = futureTime(2);
                    calibrateIndex++;
                }
                break;
            }

            case 1: {
                steering.setPower(0);
                if (System.nanoTime() > waitTime) {
                    calibrateIndex++;
                }
                break;
            }


            case 2: {
                steering.setPower(1);
                if (steering.getCurrent(CurrentUnit.AMPS) > STEERING_STALL_AMPS || gamepad1.a) {
                    maxRightSteeringTicks = steering.getCurrentPosition(); //store rightmost ticks based on stall (should be ~11k)
                    waitTime = futureTime(2);
                    calibrateIndex++;
                }
                break;
            }

            case 3: {
                steering.setPower(0);
                if (System.nanoTime() > waitTime) {
                    calibrateIndex++;
                }
                break;
            }

            case 4: {
                steering.setPower(1);
                centerSteeringTicks = maxLeftSteeringTicks + ((Math.abs(maxLeftSteeringTicks) + Math.abs(maxRightSteeringTicks)) / 2); //assume center is halfway between left max and right max
                steering.setTargetPosition(centerSteeringTicks);
                steering.setMode(DcMotor.RunMode.RUN_TO_POSITION); //switch back to runtoposition to go to center
                if (steering.getCurrentPosition() == steering.getTargetPosition()) {
                    //go to center and set center to 0, keeping left negative and right positive
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
            //verify that steering was calibrated with encoder cables plugged in
            case 5: {
                if (maxRightSteeringTicks != 0 && maxLeftSteeringTicks != 0) {
                    counterBrake.setPower(1);
                    calibrateIndex++;
                } else {
                    calibrateFailed = true;
                }

                break;
            }
            case 6: {
                //drive counterbrake motor until string is taut (verified by x button or amperage jump)
                if (counterBrake.getCurrent(CurrentUnit.AMPS) > COUNTERBRAKE_TENSION_AMPS || gamepad1.x) {
                    counterBrake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    counterBrake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    counterBrake.setVelocity(COUNTERBRAKE_CALIBRATE_VELOCITY);
                    calibrateIndex++;
                }
                break;
            }

            case 7: {
                //Max ticks are when the pedal is no longer engaged by the assembly
                //b should be pressed when driver sees the assembly pedal physically lift off of the brake
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
}
