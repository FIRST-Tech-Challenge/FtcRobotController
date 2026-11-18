package org.firstinspires.ftc.teamcode.zeroCode.finalClassess;


import android.graphics.Color;
import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;


public final class Constants {
    public static Telemetry telemetry;
    //Everything in this class is a constant that can be refred to anywhere else in the code.
    public static final class field{

        //Might want to double-check these values
        public static final int[] AprilRedHiveFlower = {30, 31, 32, 33};
        public static final int[] AprilRedHiveNoFlower = {34, 35, 36, 37};
        public static final int[] AprilBlueHiveFlower = {38, 39, 40, 41};
        public static final int[] AprilBlueHiveNoFlower = {42, 43, 44, 45};
    }

    public static final class robot{
        public enum alliance{
            BLUE,
            RED
        }


        public static double weight;

        //Configure what alliance the robot is on here:
        public static alliance robotAlliance = alliance.RED;
    }

    public static final class turret {
        //The turret left and right is dependent on what way the robot is facing
        public static final String turret1Name = "turret1";
        public static CRServo turret1;
        public static final String turret2Name = "turret2";
        public static CRServo turret2;
    }

    public static final class shooter {
        public static final String hoodName = "hood";
        public static final String leverServo1Name = "lever1";
        public static final String leverServo2Name = "lever2";
        public static Servo hood;
        public static Servo leverServo1;
        public static Servo leverServo2;
    }

    public static final class webcam {
        public static final String webcamName = "webcam";

        public static VisionPortal visionPortal;

        public static int cameraResWidth = 640;
        public static int cameraResHeight = 480;
        public static int FoV = 78;
        public static double focalLength = 660.3;
        public static double heightAboveField = 22.7;
    }

    public static final class aprilTags {
        public static AprilTagProcessor aprilTagProcessor;
    }

    public static final class colorDetection {
        public static ColorBlobLocatorProcessor colorLocator;
        public static final ColorRange pollenColorRange = new ColorRange(
                ColorSpace.YCrCb,
                //ColorSpace.HSV,
                //ORIGINAL:
                //new Scalar(20, 100, 160),
                //new Scalar(35, 255, 255)
                new Scalar(32, 128, 0),
                new Scalar(255, 200, 60) //Reduces end point for direct yellow identification.
                //Other color ranges will be determined based on the mathematical difference from Y
        );

        //ColorRange.YELLOW;

        public static final ColorRange blueNectarColorRange = new ColorRange(
                ColorSpace.YCrCb,
                new Scalar( 16,   0, 155),
                new Scalar(255, 127, 255)
        );

        public static final ColorRange redNectarColorRange = new ColorRange(
                ColorSpace.YCrCb,
                new Scalar( 32, 176,  0),
                new Scalar(255, 255, 132)
        );


    }

    public static final class pinpoint {
        public static String pinpointName = "odo";
        public static GoBildaPinpointDriver pinpointDriver;
        public static double xOffset = 77.539;
        public static double yOffset = 32.729;
        public static DistanceUnit offsetDistanceUnit = DistanceUnit.MM;
        public static GoBildaPinpointDriver.GoBildaOdometryPods OdoPodLayout = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    }

    public static final class intakes{
        public static String intakePollenName = "intakePollen";
        public static String intakeNectarName = "intakeNectar";
        public static DcMotor intakePollen;
        public static DcMotor intakeNectar;
    }

    public static final class ramp{
        public static String rampName = "ramp";
        public static Servo ramp;
        public static double rampPollenAngle = 0.0;
        public static double rampNectarAngle = 180.0;
    }

    public static final class drive{
        public static final String frontLeftMotorName = "frontLeftMotor";
        public static final String frontRightMotorName = "frontRightMotor";
        public static final String backLeftMotorName = "backLeftMotor";
        public static final String backRightMotorName = "backRightMotor";
        public static DcMotor frontLeftMotor;
        public static DcMotor frontRightMotor;
        public static DcMotor backLeftMotor;
        public static DcMotor backRightMotor;
    }

    public static final class imu {
        public static final String imuName = "imu";
        public static IMU imu;

        public static void resetYaw(){
            imu.resetYaw();
            Constants.telemetry.addLine("IMU Yaw has been reset.");
        }
    }

    public static final class hardware {

        public static boolean initializeDrive = false;
        public static boolean initializeIMU = false;
        public static boolean initializeTurret = false;
        public static boolean initializeShooter = false;
        public static boolean initializeIntakes = false;
        public static boolean initializeRamp = false;
        public static boolean initializeLimelight = false;
        public static boolean initializeWebcam = false;
        public static boolean initializeColorBlobLocator = false;
        public static boolean initializeAprilTags = false;
        public static boolean initializePinpoint = false;
        public static void resetHardwareMaps(Telemetry telemetry) {
            initializeDrive = false;
            initializeIMU = false;
            initializeTurret = false;
            initializeShooter = false;
            initializeIntakes = false;
            initializeRamp = false;
            initializeLimelight = false;
            initializeWebcam = false;
            initializeColorBlobLocator = false;
            initializeAprilTags = false;
            initializePinpoint = false;

            telemetry.addLine("Hardware Maps reset.");
            telemetry.update();

        }

        public static boolean initializeHardwareMaps(Telemetry telemetry, HardwareMap hardwareMap) {
            Constants.telemetry = telemetry;
            if (initializeDrive) {
                try {
                    drive.frontLeftMotor = hardwareMap.get(DcMotor.class, drive.frontLeftMotorName);
                    drive.frontRightMotor = hardwareMap.get(DcMotor.class, drive.frontRightMotorName);
                    drive.backLeftMotor = hardwareMap.get(DcMotor.class, drive.backLeftMotorName);
                    drive.backRightMotor = hardwareMap.get(DcMotor.class, drive.backRightMotorName);

                    drive.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                    telemetry.addLine("Drive hardware maps initialized.");
                } catch (Exception e) {
                    telemetry.addLine("Not all drive hardware maps configured.");
                    telemetry.addLine("Drive hardware maps NOT initialized.");
                    return false;
                }

            } else {
                telemetry.addLine("Drive hardware maps NOT initialized.");
            }


            if (initializeIMU) {
                try {
                    imu.imu = hardwareMap.get(IMU.class, imu.imuName);
                    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
                    // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
                    imu.imu.initialize(parameters);

                    telemetry.addLine("IMU hardware map initialized.");
                } catch (Exception e){
                    telemetry.addLine("Not all IMU hardware maps configured.");
                    telemetry.addLine("IMU hardware map NOT initialized.");
                    return false;
                }
            } else {
                telemetry.addLine("IMU hardware map NOT initialized.");
            }

            if (initializeColorBlobLocator) {
                try {
                    colorDetection.colorLocator = new ColorBlobLocatorProcessor.Builder()
                            .setTargetColorRange( colorDetection.pollenColorRange)
                            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                            .setDrawContours(true)   // Show contours on the Stream Preview
                            .setBoxFitColor(0)       // Disable the drawing of rectangles
                            .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                            .setBlurSize(5)          // Smooth the transitions between different colors in image

                            // the following options have been added to fill in perimeter holes.
                            .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                            .setErodeSize(15)        // Shrink blobs back to original size
                            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                            .build();


                    telemetry.addLine("Color Locator initialized.");
                } catch (Exception e) {
                    telemetry.addLine("Not all webcam hardware maps configured.");
                    telemetry.addLine("Color Locator NOT initialized.");
                    telemetry.addLine("Error Initilizing");
                    return false;
                }
            } else {
                telemetry.addLine("Color Locator NOT initialized.");
            }

            if (initializeAprilTags) {
                try {
                    aprilTags.aprilTagProcessor = new AprilTagProcessor.Builder()
                            .setDrawTagID(true)
                            .setDrawTagOutline(true)
                            .setDrawAxes(true)
                            .setDrawCubeProjection(true)
                            .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                            .build();
                    telemetry.addLine("April Tags initialized.");
                } catch (Exception e) {
                    telemetry.addLine("Not all April Tags configured.");
                    telemetry.addLine("April Tags NOT initialized.");
                }
            } else {
                telemetry.addLine("April Tags NOT initialized.");
            }

            if (initializeWebcam) {
                try {


                    VisionPortal.Builder builder = new VisionPortal.Builder();
                    builder.setCamera(hardwareMap.get(WebcamName.class, webcam.webcamName));
                    builder.setCameraResolution(new Size(webcam.cameraResWidth, webcam.cameraResHeight));

                    if (initializeAprilTags) {
                        builder.addProcessor(aprilTags.aprilTagProcessor);
                    }
                    if (initializeColorBlobLocator) {
                        builder.addProcessor(colorDetection.colorLocator);
                    }

                    webcam.visionPortal = builder.build();


                    telemetry.addLine("Webcam and Vision Portal initialized.");
                } catch (Exception e){
                    telemetry.addLine("Not all webcam/vision portal hardware maps configured.");
                    telemetry.addLine("Webcam and Vision Portal NOT initialized.");
                    return false;

                }
            } else {
                telemetry.addLine("Webcam and Vision Portal NOT initialized.");
            }

            if (initializeTurret) {
                try {
                    turret.turret1 = hardwareMap.get(CRServo.class, turret.turret1Name);
                    turret.turret2 = hardwareMap.get(CRServo.class, turret.turret2Name);
                    //We don't need the second turret right now
                    //turrets.turretRight = hardwareMap.get(CRServo.class, turrets.turretRightName);
                    telemetry.addLine("Turrets initialized.");
                } catch (Exception e){
                    telemetry.addLine("Not all turret hardware maps configured.");
                    telemetry.addLine("Turret NOT initialized.");
                    return false;
                }
            } else {
                telemetry.addLine("Turret NOT initialized.");
            }

            if (initializeShooter) {
                try {
                    shooter.hood = hardwareMap.get(Servo.class, shooter.hoodName);
                    shooter.leverServo1 = hardwareMap.get(Servo.class, shooter.leverServo1Name);
                    shooter.leverServo2 = hardwareMap.get(Servo.class, shooter.leverServo2Name);
                    telemetry.addLine("Shooter parts initialized.");
                } catch (Exception e){
                    telemetry.addLine("Not all shooter hardware maps configured.");
                    telemetry.addLine("Shooter parts NOT initialized.");
                    return false;
                }
            } else {
                telemetry.addLine("Shooter parts NOT initialized.");
            }

            if (initializeIntakes) {
                try {
                    intakes.intakeNectar = hardwareMap.get(DcMotor.class, intakes.intakeNectarName);
                    intakes.intakePollen = hardwareMap.get(DcMotor.class, intakes.intakePollenName);
                    telemetry.addLine("Intakes initialized.");
                } catch (Exception e) {
                    telemetry.addLine("Not all intake hardware maps configured.");
                    telemetry.addLine("Intakes not initialized.");
                    return false;
                }
            } else {
                telemetry.addLine("Intakes not initialized.");
            }

            if (initializeRamp) {
                try {
                    ramp.ramp = hardwareMap.get(Servo.class, ramp.rampName);
                    telemetry.addLine("Ramp initialized.");
                } catch (Exception e) {
                    telemetry.addLine("Ramp hardware map not configured.");
                    telemetry.addLine("Ramp not initialized.");
                    return false;
                }
            } else {
                telemetry.addLine("Ramp not initialized.");
            }

            if (initializePinpoint) {
                try {
                    pinpoint.pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, pinpoint.pinpointName);
                    pinpoint.pinpointDriver.setOffsets(pinpoint.xOffset, pinpoint.yOffset, pinpoint.offsetDistanceUnit);
                    pinpoint.pinpointDriver.setEncoderResolution(pinpoint.OdoPodLayout);
                    pinpoint.pinpointDriver.setEncoderDirections(
                            GoBildaPinpointDriver.EncoderDirection.FORWARD,
                            GoBildaPinpointDriver.EncoderDirection.REVERSED);
                    pinpoint.pinpointDriver.resetPosAndIMU();
                    telemetry.addLine("Pinpoint initialized.");
                } catch (Exception e) {
                    telemetry.addLine("Pinpoint hardware map not configured.");
                    telemetry.addLine("Pinpoint not initialized.");
                    return false;
                }
            } else {
                telemetry.addLine("Pinpoint not initialized.");
            }


            telemetry.update();

            return true;
        }

    }

}
