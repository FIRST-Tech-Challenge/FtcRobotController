package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name = "ObjectWithApril")
public class ObjectWithApril extends LinearOpMode {

    private DistanceSensor distanceSensor;
    private DcMotor fleft;
    private DcMotor fright;
    private DcMotor bright;
    private DcMotor bleft;


    @Override
    public void runOpMode() {
        int count = 0;
        String zone = "";
        int findTag;


        bleft = hardwareMap.get(DcMotor.class, "bleft");
        bright = hardwareMap.get(DcMotor.class, "bright");
        fleft = hardwareMap.get(DcMotor.class, "fleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


        bleft.setDirection(DcMotorSimple.Direction.REVERSE);
        fleft.setDirection(DcMotorSimple.Direction.REVERSE);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(640, 480))
                .build();


        waitForStart();

        //Starting Object Detection
        runMotorsTime(0.25, 1500);
        brakeMotors();
        turnLeft(0.25, 2100);

        while (opModeIsActive()) {
            // Measures Distance
            //Check Where Object Is
            if (distanceSensor.getDistance(DistanceUnit.INCH) > 25) {
                turnRight(0.25, 50);

                //Creating Zones
                count = count + 1;
                telemetry.addData("Turn_Num: ", count);

                // Zone 1
            } else if (count > 5 && count < 16) {
                zone = "Zone 1";
                zone1();
                startPark();
                findTag = 4;
                break;

                // Zone 2
            } else if (count > 17 && count < 25) {
                zone = "Zone 2";
                zone2();
                startPark();
                findTag = 5;
                break;

                // Zone 3
            } else if (count > 27) {
                zone = "Zone 3";
                zone3();
                startPark();
                findTag = 6 ;
                break;
            }
            telemetry.addData("", zone);
            telemetry.update();

            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            int detectedTagID = tag.id;
            if (tagProcessor.getDetections().size() > 0) {
                if (detectedTagID == 4) {
                    // The robot has detected AprilTag ID 4, stop moving
                    bleft.setPower(0);
                    bright.setPower(0);
                    fleft.setPower(0);
                    fright.setPower(0);

                    telemetry.addData("Detected AprilTag ID 4", true);
                    telemetry.update();

                    // Add a delay to allow time for telemetry to update and for the robot to stop
                    sleep(1000); // Adjust the sleep time as needed
                } else {
                    // AprilTag not detected, keep moving
                    bleft.setPower(-0.1);
                    bright.setPower(0.1);
                    fleft.setPower(0.1);
                    fright.setPower(-0.1);
                }
            } else{
                bleft.setPower(-0.1);
                bright.setPower(0.1);
                fleft.setPower(0.1);
                fright.setPower(-0.1);
            }
        }
    }


    // Gets Pixel on Spike 1
    public void zone1() {
        runMotorsTime(0.25, 750);
        brakeMotors();
        runMotorsTime(-0.25, 250);
        turnRight(0.5, 350);
    }

    // Gets Pixel on Spike 2
    public void zone2() {
        runMotorsTime(0.25, 1300);
        brakeMotors();
    }

    // Gets Pixel on Spike 3
    public void zone3() {
        // turnRight(0.5, 250);
        runMotorsTime(0.25, 1200);
        brakeMotors();
    }

    public void startPark() {
        runMotorsTime(-0.5, 2000);
        strafeMotorsLeft(0.25, 1000);
        runMotorsTime(0.25, 100);
        brakeMotors();
        strafeMotorsRight(0.5, 5000);
    }

    public void brakeMotors() {
        bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runMotorsTime(double power, long motorTime) {
        bleft.setPower(power);
        bright.setPower(power * 0.95);
        fright.setPower(power * 0.95);
        fleft.setPower(power);

        sleep(motorTime);

        bleft.setPower(0);
        bright.setPower(0);
        fright.setPower(0);
        fleft.setPower(0);
    }

    public void strafeMotorsRight(double power, long motorTime) {
        bleft.setPower(-power);
        bright.setPower(power);
        fright.setPower(-power);
        fleft.setPower(power);

        sleep(motorTime);

        bleft.setPower(0);
        bright.setPower(0);
        fright.setPower(0);
        fleft.setPower(0);
    }

    public void strafeMotorsLeft(double power, long motorTime) {
        bleft.setPower(power);
        bright.setPower(-power);
        fright.setPower(power);
        fleft.setPower(-power);

        sleep(motorTime);

        bleft.setPower(0);
        bright.setPower(0);
        fright.setPower(0);
        fleft.setPower(0);
    }

    public void turnRight(double power, long motorTime) {
        bleft.setPower(power);
        bright.setPower(-power);
        fright.setPower(-power);
        fleft.setPower(power);

        sleep(motorTime);

        bleft.setPower(0);
        bright.setPower(0);
        fright.setPower(0);
        fleft.setPower(0);

    }


    public void turnLeft(double power, long motorTime) {
        bleft.setPower(-power);
        bright.setPower(power);
        fright.setPower(power);
        fleft.setPower(-power);

        sleep(motorTime);

        bleft.setPower(0);
        bright.setPower(0);
        fright.setPower(0);
        fleft.setPower(0);
    }
}


