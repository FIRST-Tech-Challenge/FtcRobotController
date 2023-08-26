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

    public static double ANTI_STRESS_PROPORTIONAL = 0.9;
    boolean steeringCalibrated = false;
    boolean brakeCalibrated = false;

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

    }
    boolean isCalibratingSteering = false;
    boolean isCalibratingBrake = false;
    @Override
    public void loop() {
        telemetry();
        calibrateTelemetry();
        telemetry.update();
        if (!isCalibratingBrake && !isCalibratingSteering && steeringCalibrated && brakeCalibrated){
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
            int steeringTarget = (int) (steering.getCurrentPosition() + (-gamepad1.right_stick_x * maxSteeringTicksPerSecond));
            steeringTarget = Math.max(Math.min(steeringTarget,maxRightSteeringTicks),maxLeftSteeringTicks);
            steering.setTargetPosition(steeringTarget);

            //if deadman's switch (on second gamepad) is intentionally depressed, allow acceleration and apply counterBrake
            if (gamepad2.left_trigger > .5) {
                counterBrake.setPower(1);
                counterBrake.setTargetPosition((int) (gamepad1.right_trigger * counterBrakeMaxTicks));
                //accelerator.setPower(gamepad1.right_trigger);
            } else {
                counterBrake.setPower(0);
            }
        } else {

        }
        if(gamepad1.y){
            isCalibratingSteering = true;
        }
        if(isCalibratingSteering){
            isCalibratingSteering = !calibrateSteering();
        }

        if(gamepad1.x){
            isCalibratingBrake = true;
        }
        if(isCalibratingBrake){
            isCalibratingBrake = !calibrateBrake();
        }
    }

    @Override
    public void stop() {
        counterBrake.setPower(0); //removes counterBrake (holds brake)
        steering.setTargetPosition(steering.getCurrentPosition()); //locks steering
    }

    public void telemetry() {
        if(steeringCalibrated && !calibrateFailed) {
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

    int calibrateBrakeIndex = 0;
    public boolean calibrateBrake(){
        switch (calibrateBrakeIndex){
            case 0: {
                brakeCalibrated = false;
                counterBrake.setPower(1);
                calibrateBrakeIndex++;
                break;
            }
            case 1: {
                //drive counterbrake motor until string is taut (verified by x button or amperage jump)
                if (counterBrake.getCurrent(CurrentUnit.AMPS) > COUNTERBRAKE_TENSION_AMPS) {
                    counterBrake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    counterBrake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    counterBrake.setVelocity(COUNTERBRAKE_CALIBRATE_VELOCITY);
                    calibrateBrakeIndex++;
                }
                break;
            }

            case 2: {
                //Max ticks are when the pedal is no longer engaged by the assembly
                //b should be pressed when driver sees the assembly pedal physically lift off of the brake
                if (gamepad1.b) {
                    counterBrakeMaxTicks = counterBrake.getCurrentPosition();
                    counterBrake.setTargetPosition(0);
                    counterBrake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    calibrateBrakeIndex++;
                }
                break;
            }

            case 3: {
                brakeCalibrated = true;
                calibrateBrakeIndex = 0;
                return true;
            }
        }
        return false;
    }

    public boolean calibrateSteering() {
        switch (calibrateIndex) {
            case 0: {
                steeringCalibrated = false;
                steering.setPower(-1);
                if (steering.getCurrent(CurrentUnit.AMPS) > STEERING_STALL_AMPS || gamepad1.a) {
                    maxLeftSteeringTicks = 0; //store leftmost ticks based on stall (should be ~-11k)
                    steering.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    steering.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    waitTime = futureTime(0.5);
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
                    maxRightSteeringTicks = (int)(steering.getCurrentPosition()); //store rightmost ticks based on stall (should be ~11k)
                    waitTime = futureTime(0.5);
                    calibrateIndex++;
                }
                break;
            }

            case 3: {
                steering.setPower(0);
                if (System.nanoTime() > waitTime) {
                    calibrateIndex++;
                }
                steeringCalibrated = true;
                break;
            }

            case 4: {
                steering.setPower(1);
                centerSteeringTicks = maxRightSteeringTicks / 2; //assume center is halfway between left max and right max
                steering.setTargetPosition(centerSteeringTicks);
                steering.setMode(DcMotor.RunMode.RUN_TO_POSITION); //switch back to runtoposition to go to center
                if (steering.getCurrentPosition() == steering.getTargetPosition()) {
                    //go to center and set center to 0, keeping left negative and right positive
                    steering.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    steering.setTargetPosition(0);
                    steering.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    maxLeftSteeringTicks = (int)(-centerSteeringTicks * ANTI_STRESS_PROPORTIONAL);
                    maxRightSteeringTicks = (int)(centerSteeringTicks * ANTI_STRESS_PROPORTIONAL);
                    centerSteeringTicks = 0;
                    calibrateIndex++;
                }
                break;
            }
            case 5: {
                steeringCalibrated = true;
                calibrateIndex = 0;
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
