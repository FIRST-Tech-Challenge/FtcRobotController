package org.firstinspires.ftc.teamcode.robots.r2v2;
import static org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils.diffBetweenAnglesDeg;
import static org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils.map;
import static org.firstinspires.ftc.teamcode.util.utilMethods.boundDouble;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.r2v2.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.r2v2.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.robots.r2v2.vision.provider.R2V2ConeDetectorProvider;
import org.firstinspires.ftc.teamcode.robots.r2v2.vision.Target;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import java.util.Map;

@Config("R2V2")
@TeleOp(name = "R2V2", group = "Challenge")
public class R2V2 extends OpMode {

    //motors
    DcMotorEx counterBrake, steering, accelerator;
    Servo cameraServo;
    //variables
    double cameraErrorCorrection = 0;
    double cameraServoTarget = .5;
    static double camP = .1; //proportional multiplier for error correction

    long currentSteeringTicks;
    long waitTime;
    boolean calibrateFailed = false;
    public static int maxSteeringTicksPerSecond = 500;
    public int maxRightSteeringTicks, maxLeftSteeringTicks, centerSteeringTicks;
    int calibrateIndex = 0;
    int coneStage;
    FtcDashboard dashboard;
    public static double COUNTERBRAKE_TENSION_AMPS = 3.5;
    public static int COUNTERBRAKE_CALIBRATE_VELOCITY = 150;

    public int counterBrakeMaxTicks;
    public static double STEERING_STALL_AMPS = 4.5;

    public static double ANTI_STRESS_PROPORTIONAL = 0.9;
    boolean steeringCalibrated = false;
    boolean brakeCalibrated = false;

    int steeringTarget = 0;

    R2V2ConeDetectorProvider coneVision = null;
    List<Target> uniqueCones = null;
    List<Target> frameCones = null;
    Target currentTarget = null;
    public enum ControlMode{
        FOLLOW_CONE,
        GPS,
        MANUAL,
        CALIBRATE
    }

    ControlMode mode = ControlMode.CALIBRATE;
    @Override
    public void init() {
        mode = ControlMode.FOLLOW_CONE;//when testing vision pipeline
        boolean initialized = initMotors();
        dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();
        //get a Telemetry object that works for both dash and driver stations
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();
        calibrateIndex = 0;
        createVisionProvider(1);
        visionProvider.initializeVision(hardwareMap);
        if (!initialized)
            stop();
    }

    public VisionProvider visionProvider;

    public void createVisionProvider(int visionProviderIndex) {
        try {
            visionProvider = VisionProviders.VISION_PROVIDERS[visionProviderIndex].newInstance();
        } catch(IllegalAccessException | InstantiationException e) {
            throw new RuntimeException("Error while instantiating vision provider");
        }
    }



    @Override
    public void init_loop() {

    }
    boolean isCalibratingSteering = false;
    boolean isCalibratingBrake = false;


    @Override
    public void loop() {

        if(gamepad1.dpad_down) {
            mode = ControlMode.CALIBRATE;
        }
        if(gamepad1.dpad_up) {
            mode = ControlMode.MANUAL;
        }
        if(gamepad1.dpad_right) {
            mode = ControlMode.FOLLOW_CONE;
            coneStage = 0;
        }
        if(gamepad1.dpad_left) {
            mode = ControlMode.GPS;
        }
        if(mode.equals(ControlMode.MANUAL)) {
            if (!isCalibratingBrake && !isCalibratingSteering && steeringCalibrated && brakeCalibrated) {
                //emergency stop button
                //locks steering and disables counterbrake and accelerator to be backdriven
                //todo IMPORTANT - this test should be enforced regardless of mode ASAP
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
                steeringTarget = (int) (steering.getCurrentPosition() + (-gamepad1.right_stick_x * maxSteeringTicksPerSecond));

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
        }
        else if(mode.equals(ControlMode.CALIBRATE)) {
            if (gamepad1.y) {
                isCalibratingSteering = true;
            }
            if (isCalibratingSteering) {
                isCalibratingSteering = !calibrateSteering();
            }

            if (gamepad1.x) {
                isCalibratingBrake = true;
            }
            if (isCalibratingBrake) {
                isCalibratingBrake = !calibrateBrake();
            }
        }
        else if(mode.equals(ControlMode.FOLLOW_CONE))
        {
            coneVision = (R2V2ConeDetectorProvider) visionProvider;
            coneVision.update();

            frameCones = coneVision.getFrameDetections();
            //currentTarget = ((DPRGCanDetectorProvider) visionProvider).GetNearest(frameCans, new Vector2d(0, 0));//uses the robots location we dont know our location so Im using vector 0, 0

            if(frameCones.size() > 0) {
                currentTarget = frameCones.get(0);

                for (Target cone : frameCones) {
                    if (cone.getHeightPixels() > currentTarget.getHeightPixels()) {
                        currentTarget = cone;
                    }
                }
                cameraErrorCorrection = -diffBetweenAnglesDeg(currentTarget.getCameraHeading(), 0);
                //map the range of +/- 32 degrees FOV to a much smaller correction range equivalent to about 1/4 of a servo's movement range
                cameraErrorCorrection = map(cameraErrorCorrection,-32,32,-.125,.125);


                cameraServoTarget -= (cameraErrorCorrection * camP);


            }
            //experimentally, a servo position value of .5 +/- .25 represents the useful range in that the camera would be looking more at the interior of the vehicle outside of that range
            //so constrain to that range
            cameraServoTarget = boundDouble(cameraServoTarget, .25, .75);
            cameraServo.setPosition(cameraServoTarget);

            //send the camera angle as a steering wheel correction
            //map the range of the tracking camera servo to the range of the target steering ticks
            //todo - we should change this to relating angles instead of target positions
            steeringTarget = (int)map(cameraServoTarget, .25, .75, maxLeftSteeringTicks, maxRightSteeringTicks);


            //if the vision target is too large (experimentally determine), then apply the brake automatically
            //todo this needs a LOT of work to convert the target height or size to a real distance value and do gradual braking
            if (currentTarget.getAreaPixels()>4000)
                counterBrake.setTargetPosition(0); //full braking at the moment



        }
        update();
    }

    //todo - we really should have done our normal architecture where this update function is in a robot subsystem
    public void update(){

        steeringTarget = Math.max(Math.min(steeringTarget, maxRightSteeringTicks), maxLeftSteeringTicks);
        steering.setTargetPosition(steeringTarget);

        telemetry();
        calibrateTelemetry();
        telemetry.update();

    }

    @Override
    public void stop() {
        counterBrake.setPower(0); //removes counterBrake (holds brake)
        steering.setTargetPosition(steering.getCurrentPosition()); //locks steering
    }

    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName, TelemetryPacket packet) {
        telemetry.addLine(telemetryName);
        packet.addLine(telemetryName);
        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            packet.addLine(line);
            telemetry.addLine(line);
        }

        telemetry.addLine();
        packet.addLine("");
    }

    public void telemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        if(visionProvider != null) {
            dashboard.sendImage(visionProvider.getDashboardImage());
            telemetry.addData("Number of Targets", ((R2V2ConeDetectorProvider) visionProvider).getFrameDetections().size());
        }
        if(currentTarget != null){
            telemetry.addData("Centroid:\t", currentTarget.getCentroid().getX());
            telemetry.addData("Centroid:\t", currentTarget.getCentroid().getY());
        }

        Map<String, Object> visionTelemetryMap = visionProvider.getTelemetry(true);
        visionTelemetryMap.put("Backend",
                Misc.formatInvariant("%s (%s)",
                        org.firstinspires.ftc.teamcode.robots.r2v2.vision.VisionProviders.VISION_PROVIDERS[1].getSimpleName(),
                        visionProvider != null ?
                                "finalized" :
                                System.currentTimeMillis() / 500 % 2 == 0 ? "**NOT FINALIZED**" : "  NOT FINALIZED  "
                )
        );
        handleTelemetry(visionTelemetryMap, visionProvider.getTelemetryName(), packet);
        packet.put("Cam Error Corrector",cameraErrorCorrection);

        telemetry.addData("Mode:\t", mode);

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
            telemetry.addData("Camera Servo Target:\t", cameraServoTarget);
            telemetry.addData("Camera Correction:\t", cameraErrorCorrection);

        }
        dashboard.sendTelemetryPacket(packet);
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
                //todo - change this to use pressure sensors
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
            cameraServo = this.hardwareMap.get(Servo.class, "cameraServo");
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
