package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.kdrobot.KDRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

    /*
     * This OpMode illustrates the basics of TensorFlow Object Detection,
     * including Java Builder structures for specifying Vision parameters.
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
     */
    @TeleOp
    public class redBackstage extends LinearOpMode {

        private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

        /**
         * The variable to store our instance of the TensorFlow Object Detection processor.
         */
        private TfodProcessor tfod;
        private static final String TFOD_MODEL_ASSET = "redTeamPropIL.tflite";
        private static final String[] LABELS = {
                "RedProp",
        };

        /**
         * The variable to store our instance of the vision portal.
         */
        private VisionPortal visionPortal;
        String labelName;
        double middleOfImageWidth;
        double teamPropPosition;
        double stepNumber = 1;
        double leftSpeed = 0;
        double rightSpeed = 0;
        double driveTrainSpeed = 0;
        double correction;
        double timer;


        @Override
        public void runOpMode() {
            int spikeNumber = 0;
            initTfod();
            boolean detectionActive = true;
            spikeNumber = getSpikeNumber();
            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            KDRobot robot = new KDRobot();
            robot.init(hardwareMap, telemetry);
            DcMotor backLeft = hardwareMap.get(DcMotor.class,"backLeft");
            Servo pixelPusher = hardwareMap.get(Servo.class,"pixelPusher");
            DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class,"armMotor");
            IMU imu = hardwareMap.get(IMU.class,"imu");
            IMU.Parameters myIMUparameters;
            myIMUparameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );
            imu.initialize(myIMUparameters);
            imu.resetYaw();
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            waitForStart();
            while (opModeIsActive()) {
                //telemetry.addData("spikeNumber",spikeNumber);
                //telemetry.addData("ddet",detectionActive);
                //telemetry.update();
                double driveTrainPoistion = backLeft.getCurrentPosition();
                if (detectionActive) {
                    if (spikeNumber == 0) {
                        spikeNumber = getSpikeNumber();
                    }
                    else {
                        detectionActive = false;
                    }

                }

                if (spikeNumber == 1) {
                    redLeft(robot,backLeft,imu,yawAngle,driveTrainPoistion, pixelPusher, armMotor);
                }
                if (spikeNumber == 2) {
                    redCenter(robot,backLeft,imu,yawAngle,driveTrainPoistion, pixelPusher, armMotor);
                }
                if (spikeNumber == 3) {
                    redRight(robot,backLeft,imu,yawAngle,driveTrainPoistion, pixelPusher, armMotor);
                }


            }

            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();

        }   // end runOpMode()
        private void initTfod() {
            // Create the TensorFlow processor by using a builder.
            tfod = new TfodProcessor.Builder()
                    .setModelAssetName(TFOD_MODEL_ASSET)
                    .setModelLabels(LABELS)
                    //.setIsModelTensorFlow2(true)
                    //.setIsModelQuantized(true)
                    //.setModelInputSize(480)
                    //.setModelAspectRatio(16.0 / 9.0)

                    .build();

            // Create the vision portal by using a builder.
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Set the camera (webcam vs. built-in RC phone camera).
            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }

            // Choose a camera resolution. Not all cameras support all resolutions.
            //builder.setCameraResolution(new Size(640, 480));

            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            //builder.enableCameraMonitoring(true);

            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            //builder.setAutoStopLiveView(false);

            builder.addProcessor(tfod);


            visionPortal = builder.build();

            //tfod.setMinResultConfidence(50f);

        }   // end method initTfod()
        private int getSpikeNumber() {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            // Step through the list of recognitions and display info for each one.
            int spikeNumber = 0;
            if (timer < 5000) {
                currentRecognitions = tfod.getRecognitions();
                for (Recognition recognition : currentRecognitions) {
                    middleOfImageWidth = (double) recognition.getImageWidth() / 2;
                    teamPropPosition = (recognition.getLeft() + recognition.getRight()) / 2;
                    telemetry.addData("Pixel", "Recognized");
                    if (teamPropPosition < middleOfImageWidth) {
                        return 1;
                    } else if (teamPropPosition > middleOfImageWidth) {
                        return 2;
                    }
                }
                sleep(500);
                telemetry.addData("timer", timer);
                telemetry.addData("current", currentRecognitions.size());
                telemetry.update();
                timer = timer + 500;
            }
            else {
                return 3;
            }
            return 0;
        }
        private void redLeft (KDRobot robot, DcMotor backLeft, IMU imu, double yawAngle, double driveTrainPoistion, Servo pixelPusher,  DcMotorEx armMotor){
            driveTrainPoistion = backLeft.getCurrentPosition();
            driveTrainSpeed = 0.27;
            yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (stepNumber == 1){
                armMotor.setPower(0.35);
                sleep(500);
                armMotor.setPower(0.08);
                stepNumber = 1.5;
            }
            if (1500 >= driveTrainPoistion && stepNumber == 1.5) {
                correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftSpeed = -15 + correction;
                rightSpeed = -15 -correction;
                robot.setWheelPower(leftSpeed/-100,rightSpeed/-100,leftSpeed/-100,rightSpeed/-100);
                //robot.setDriveTrainSpeed(0.15);
                //robot.moveDriveTrain("FORWARD");
                telemetry.addData("driveTrainPosition",driveTrainPoistion);
                telemetry.update();
            }
            else if (stepNumber == 1.5){
                robot.stopDriveTrain();
                stepNumber = 2;
            }
            if (stepNumber == 2) {
                robot.setDriveTrainSpeed(0.27);
                robot.moveDriveTrain("STRAFE_RIGHT");
                sleep(1550);
                robot.stopDriveTrain();
                stepNumber = 3;
            }
            if (stepNumber == 3) {
                if (yawAngle >= 89) {
                    robot.stopDriveTrain();
                    telemetry.addData("yawAngle > 90","STOP ACTION");
                    telemetry.update();
                    stepNumber = 4;
                } else if (yawAngle <= 90){
                    robot.setWheelPower(-0.15, 0.15, -0.15,0.15);
                    telemetry.addData("keep rotating",yawAngle);
                    telemetry.update();
                }
            }
            if (stepNumber == 4) {
                pixelPusher.setPosition(0.7);
                sleep(500);
                stepNumber= 5;
            }
            if (stepNumber == 5) {
                robot.moveDriveTrain("BACKWARD");
                sleep(500);
                robot.stopDriveTrain();
                stepNumber = 6;
            }
            if (stepNumber==6) {
                robot.moveDriveTrain("STRAFE_LEFT");
                sleep(1000);
                robot.stopDriveTrain();
                stepNumber= 7;
            }
            if (stepNumber == 7) {
                imu.resetYaw();
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                driveTrainPoistion = backLeft.getCurrentPosition();
                stepNumber = 8;
            }
            if (-1100 <= driveTrainPoistion && stepNumber == 8) {
                correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftSpeed = -15 - correction;
                rightSpeed = -15 + correction;
                robot.setWheelPower(leftSpeed/100,rightSpeed/100,leftSpeed/100,rightSpeed/100);
                telemetry.addData("driveTrainPosition",driveTrainPoistion);
                telemetry.update();
            }
            else if (stepNumber == 8){
                robot.stopDriveTrain();
                stepNumber = 9;
            }
            if (stepNumber ==9) {
                armMotor.setPower(0.35);
                sleep(1000);
                armMotor.setPower(0);
                stepNumber = 9;
            }



        }
        private void redCenter (KDRobot robot, DcMotor backLeft, IMU imu, double yawAngle, double driveTrainPoistion, Servo pixelPusher, DcMotorEx armMotor){
            driveTrainPoistion = backLeft.getCurrentPosition();
            driveTrainSpeed = 0.27;
            yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (stepNumber == 1){
                armMotor.setPower(0.35);
                sleep(500);
                armMotor.setPower(0.08);
                stepNumber = 1.5;
            }
            if (stepNumber == 1.5) {
                robot.setDriveTrainSpeed(0.4);
                robot.moveDriveTrain("STRAFE_RIGHT");
                sleep(100);
                robot.stopDriveTrain();
                stepNumber = 2;
            }
            if (1175 >= driveTrainPoistion && stepNumber == 2) {
                correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftSpeed = -15 + correction;
                rightSpeed = -15 -correction;
                robot.setWheelPower(leftSpeed/-100,rightSpeed/-100,leftSpeed/-100,rightSpeed/-100);
                //robot.setDriveTrainSpeed(0.15);
                //robot.moveDriveTrain("FORWARD");
                telemetry.addData("driveTrainPosition",driveTrainPoistion);
                telemetry.update();
            }
            else if (stepNumber == 2){
                robot.stopDriveTrain();
                stepNumber = 3;
            }
            if (stepNumber == 3) {
                pixelPusher.setPosition(0.7);
                sleep(1000);
                stepNumber = 4;
            }
            if (stepNumber == 4) {
                robot.setWheelPower(-driveTrainSpeed,-driveTrainSpeed,-driveTrainSpeed,-driveTrainSpeed);
                sleep(500);
                robot.stopDriveTrain();
                stepNumber = 5;
            }
            if (stepNumber == 5) {
                if (yawAngle >= 89) {
                    robot.stopDriveTrain();
                    telemetry.addData("yawAngle > 90","STOP ACTION");
                    telemetry.update();
                    stepNumber = 6;
                } else if (yawAngle <= 90){
                    robot.setWheelPower(-driveTrainSpeed, driveTrainSpeed, -driveTrainSpeed,driveTrainSpeed);
                    telemetry.addData("keep rotating",yawAngle);
                    telemetry.update();
                }
            }
            if (stepNumber == 6) {
                imu.resetYaw();
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                driveTrainPoistion = backLeft.getCurrentPosition();
                stepNumber = 7;
            }
            if (-1000 <= driveTrainPoistion && stepNumber == 7) {
                correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftSpeed = -15 - correction;
                rightSpeed = -15 + correction;
                robot.setWheelPower(leftSpeed/100,rightSpeed/100,leftSpeed/100,rightSpeed/100);
                telemetry.addData("driveTrainPosition",driveTrainPoistion);
                telemetry.update();
            }
            else if (stepNumber == 7){
                robot.stopDriveTrain();
                stepNumber = 8;
            }
            if (stepNumber == 8) {
                robot.moveDriveTrain("STRAFE_RIGHT");
                sleep (600);
                stepNumber = 9;
            }
            if (stepNumber == 9) {
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                driveTrainPoistion = backLeft.getCurrentPosition();
                stepNumber = 10;
            }
            if (-1000 <= driveTrainPoistion && stepNumber == 10) {
                correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftSpeed = -15 - correction;
                rightSpeed = -15 + correction;
                robot.setWheelPower(leftSpeed/100,rightSpeed/100,leftSpeed/100,rightSpeed/100);
                telemetry.addData("driveTrainPosition",driveTrainPoistion);
                telemetry.update();
            }
            else if (stepNumber == 10){
                robot.stopDriveTrain();
                stepNumber = 11;
            }
            if (stepNumber == 11) {
                robot.setDriveTrainSpeed(0.2);
                robot.moveDriveTrain("BACKWARD");
                sleep(1000);
                robot.stopDriveTrain();
                stepNumber = 12;
            }
            if (stepNumber == 12) {
                armMotor.setPower(0.5);
                sleep(2000);
                armMotor.setPower(0);
                sleep(500);
                armMotor.setPower(-0.3);
                sleep(1000);
                armMotor.setPower(0);
                stepNumber = 13;
            }



        }
        private void redRight (KDRobot robot, DcMotor backLeft, IMU imu, double yawAngle, double driveTrainPoistion, Servo pixelPusher, DcMotorEx armMotor){
            driveTrainPoistion = backLeft.getCurrentPosition();
            driveTrainSpeed = 0.27;
            yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (stepNumber == 1){
                armMotor.setPower(0.35);
                sleep(500);
                armMotor.setPower(0.08);
                stepNumber = 1.5;
            }
            if (stepNumber == 1.5) {
                robot.setDriveTrainSpeed(0.4);
                robot.moveDriveTrain("STRAFE_RIGHT");
                sleep(900);
                robot.stopDriveTrain();
                stepNumber = 2;
            }
            if (175 >= driveTrainPoistion && stepNumber == 2) {
                correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftSpeed = -15 + correction;
                rightSpeed = -15 -correction;
                robot.setWheelPower(leftSpeed/-100,rightSpeed/-100,leftSpeed/-100,rightSpeed/-100);
                //robot.setDriveTrainSpeed(0.15);
                //robot.moveDriveTrain("FORWARD");
                telemetry.addData("driveTrainPosition",driveTrainPoistion);
                telemetry.update();
            }
            else if (stepNumber == 2){
                robot.stopDriveTrain();
                stepNumber = 3;
            }
            if (stepNumber == 3) {
                robot.moveDriveTrain("FORWARD");
                sleep(0);
                robot.stopDriveTrain();
                stepNumber = 3.5;
            }
            if (stepNumber == 3.5) {
                pixelPusher.setPosition(0.7);
                sleep(1000);
                stepNumber = 4;
            }
            if (stepNumber == 4) {
                robot.setWheelPower(-driveTrainSpeed,-driveTrainSpeed,-driveTrainSpeed,-driveTrainSpeed);
                sleep(500);
                robot.stopDriveTrain();
                stepNumber = 5;
            }
            if (stepNumber == 5) {
                if (yawAngle >= 89) {
                    robot.stopDriveTrain();
                    telemetry.addData("yawAngle > 90","STOP ACTION");
                    telemetry.update();
                    stepNumber = 6;
                } else if (yawAngle <= 90){
                    robot.setWheelPower(-driveTrainSpeed, driveTrainSpeed, -driveTrainSpeed,driveTrainSpeed);
                    telemetry.addData("keep rotating",yawAngle);
                    telemetry.update();
                }
            }
            if (stepNumber == 6) {
                imu.resetYaw();
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                driveTrainPoistion = backLeft.getCurrentPosition();
                stepNumber = 7;
            }
            if (-500 <= driveTrainPoistion && stepNumber == 7) {
                correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftSpeed = -15 - correction;
                rightSpeed = -15 + correction;
                robot.setWheelPower(leftSpeed/100,rightSpeed/100,leftSpeed/100,rightSpeed/100);
                telemetry.addData("driveTrainPosition",driveTrainPoistion);
                telemetry.update();
            }
            else if (stepNumber == 7){
                robot.stopDriveTrain();
                stepNumber = 8;
            }
            if (stepNumber == 8) {
                robot.moveDriveTrain("STRAFE_RIGHT");
                sleep (1000);
                stepNumber = 9;
            }
            if (stepNumber == 9) {
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                driveTrainPoistion = backLeft.getCurrentPosition();
                stepNumber = 10;
            }
            if (-800 <= driveTrainPoistion && stepNumber == 10) {
                correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftSpeed = -15 - correction;
                rightSpeed = -15 + correction;
                robot.setWheelPower(leftSpeed/100,rightSpeed/100,leftSpeed/100,rightSpeed/100);
                telemetry.addData("driveTrainPosition",driveTrainPoistion);
                telemetry.update();
            }
            else if (stepNumber == 10){
                robot.stopDriveTrain();
                stepNumber = 11;
            }
            if (stepNumber == 11) {
                armMotor.setPower(0.35);
                sleep(3000);
                armMotor.setPower(0);
                stepNumber = 12;
            }


        }

    }


