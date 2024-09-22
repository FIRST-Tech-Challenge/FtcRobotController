package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class TimeBasedAutoBase extends LinearOpMode {

    public final ElapsedTime runtime = new ElapsedTime();
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    DcMotor armMotor = null;
    Servo clawServoR,clawServoL ;
    DcMotor armMotor2;
    public DcMotor frontLeftMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backRightMotor = null;
    IMU imu;
    @Override
    public void runOpMode() {

        try {
            initLocal();
            onStart();
            goToBackStage();
            backStageStuff();
            park();
            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(500);
        } catch (Exception ex) {
            ex.printStackTrace();
        } finally {
            if (visionPortal != null) {
                visionPortal.close();
            }
        }

    }


    public void park() {
        log("do nothing", "do nothing", 10);
    }
    public void goToBackStage() {
        log("do nothing", "do nothing", 10);
    }

    private void backStageStuff() {

        log("arm current position", armMotor.getCurrentPosition(), 500);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(.8);
        armMotor.setTargetPosition(4000);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        while (runtime.seconds() < 2.0 && armMotor.isBusy()) {
            idle();
        }

        log("arm current position", armMotor2.getCurrentPosition(), 500);

        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setPower(.4);
        armMotor2.setTargetPosition(1150);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        while (runtime.seconds() < 2.0 && armMotor2.isBusy()) {
            idle();
        }

        //log("claw position position", clawServo.getPosition(), 100);

        clawServoR.setPosition(0.6);
        clawServoL.setPosition(0.3);

        sleep(1000);

        clawServoR.setPosition(0.05);
        clawServoL.setPosition(0.70);

        //log("arm current position", armMotor2.getCurrentPosition(), 500);
       // log("claw position position", clawServo.getPosition(), 500);

    }

    private void initLocal() {
        //initTfod();

        initChassis();

        initNonChassis();
    }

    public void log(String topic, Object message, int sleepTime){
        telemetry.addData(topic, message);
        telemetry.update();
        sleep(sleepTime);
    }

    private void onStart() {
        log("Status", "Ready to run", 0);
        // Wait for the game to start (driver presses PLAY)
       // initIMUmodule();
        waitForStart();
        log("Status", "Started", 100);

        log("arm current position", armMotor.getCurrentPosition(), 500);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(.8);
        armMotor.setTargetPosition(1300);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        while (runtime.seconds() < 2.0 && armMotor.isBusy()) {
            idle();
        }

    }

    /**
     * Takes the actual values (for teleop) or the equivalent values(for auton) and moves the macanum wheels accordingly
     * @param x  How much is the left joystick is pressed along x axis. Values have to be between 1 and -1
     * @param y How much is the left joystick is pressed along y axis. Values have to be between 1 and -1
     * @param turn How much is the right joystick is pressed along x axis. Values have to be between 1 and -1
     */
    public void move(double x, double y, double turn){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        double frontLeftPower = (y + x + turn) / denominator;
        double backLeftPower = (y - x + turn) / denominator;
        double frontRightPower = (y - x - turn) / denominator;
        double backRightPower = (y + x - turn) / denominator;

        double powerRatio = 0.3;
        setChassisPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower, powerRatio);
    }

    public void setChassisPower(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower, double powerRatio) {
        telemetry.addData("desired power ", powerRatio * frontLeftPower);
        telemetry.addData("desired power ", powerRatio * backLeftPower);
        telemetry.addData("desired power ", powerRatio * frontRightPower);
        telemetry.addData("desired power ", powerRatio * backRightPower);
        telemetry.update();

        frontLeftMotor.setPower(powerRatio * frontLeftPower);
        backLeftMotor.setPower(powerRatio * backLeftPower);
        frontRightMotor.setPower(powerRatio * frontRightPower);
        backRightMotor.setPower(powerRatio * backRightPower);
    }


    public void stopChassis() {
        log("desired power", 0, 0);
        telemetry.addData("desired power ", 0);
        telemetry.update();
        sleep(100);

        setChassisPower(0,0,0,0,0);
    }

    private void initNonChassis() {
        armMotor = hardwareMap.dcMotor.get("armMotor");
        clawServoR = hardwareMap.servo.get("clawServo");
        clawServoL = hardwareMap.servo.get("clawServo2");
        armMotor2 = hardwareMap.dcMotor.get("armMotor2");
        clawServoR.setPosition(0.05);
        clawServoL.setPosition(0.70);
    }

    private void initChassis() {
        frontLeftMotor = hardwareMap.dcMotor.get(DeviceNames.MOTOR_FRONT_LEFT);
        backLeftMotor = hardwareMap.dcMotor.get(DeviceNames.MOTOR_BACK_LEFT);
        frontRightMotor = hardwareMap.dcMotor.get(DeviceNames.MOTOR_FRONT_RIGHT);
        backRightMotor = hardwareMap.dcMotor.get(DeviceNames.MOTOR_BACK_RIGHT);
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void dropYellowPixel() {
        armMotor.setTargetPosition(100);
        armMotor2.setTargetPosition(-50);
        clawServoR.setPosition(0.6);
        clawServoL.setPosition(0.3);
    }

    private void navigateToBackStage() {
//        wheels.rotateLeft90(100);
//        wheels.goForward(100);
//        wheels.stop();
    }

    private void backout() {
//        wheels.back(100);
    }

    private void placePurplePixel() {
        armMotor2.setTargetPosition(-1000);
        clawServoR.setPosition(0.8);
        armMotor2.setTargetPosition(5);
        clawServoR.setPosition(0.3);
    }

    /**
     * Tell which stripe has pixel (1 for left, 2 for center, 3 for right)
     * <p>
     * If cannot detect pixel, then return 1.
     *
     * @return
     */
    private int getStripe() {

        int stripe = 0;

//        visionPortal.resumeStreaming();
        telemetry.addData(" > Camera Status", visionPortal.getCameraState());
        telemetry.update();
        sleep(500);
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        //      visionPortal.stopStreaming();
        telemetry.addData(" > Camera Status", visionPortal.getCameraState());
        telemetry.update();
        sleep(1000);

        telemetry.addData(" > currentRecognitions.size()", currentRecognitions.size());
        telemetry.update();
        sleep(1000);

        if (currentRecognitions.size() == 1) {
            //find location, move robot and place purple pixel

            Recognition recognition = currentRecognitions.get(0);
            telemetry.addData("recognition",
                    String.format("recognition right %s, left %s, top %s,bottom %s ", recognition.getRight(),
                            recognition.getLeft(), recognition.getTop(), recognition.getBottom()));
            telemetry.update();
            sleep(1000);

            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            sleep(1000);

            visionPortal.stopStreaming();
        } else {
            telemetry.addData("recognition", "none");
            sleep(1000);
            telemetry.update();
            //move robot to backstage
        }

        telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
        telemetry.update();

        return stripe;
    }
    private void initIMUmodule(){
        imu = hardwareMap.get(IMU.class, "imu");

        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Loop and update the dashboard
        while (!isStopRequested()) {

            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

            // Check to see if heading reset is requested
            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
            }

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
        }
    }



    public boolean inRange(double distance, int start, int end) {
        return distance > start && distance < end;
    }

    private void initTfod() {

        tfod = TfodProcessor.easyCreateWithDefaults();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tfod)
                .build();

        telemetry.addData("vision portal", "start");
        telemetry.update();
        sleep(1000);

    }

}


