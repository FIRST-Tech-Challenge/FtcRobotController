/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;

@Autonomous(name="jerry_right")
public class AutonFinal extends LinearOpMode
{
    final static double ARM_HOME = 0.0;
    final static double ARM_MIN_RANGE = 0.0;
    final static double ARM_MAX_RANGE = 1.0;
    final static double HEXMOTOR_TPR = 288;
    final static double GOBILDA_TPR = 751.8; //5202
    final static double GEAR_REDUCTION = 1;
    final static double TICKS_PER_DEGREE = (HEXMOTOR_TPR * GEAR_REDUCTION) / 360;
    double X=0;
    double k =1;//square distance for run to pos
    //ticks = pulses, cycles = ticks * 4

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(1/00);


        Project1Hardware robot = new Project1Hardware();
        MecanumDrive drivetrain  = new MecanumDrive(robot);

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.imu1.resetYaw();

        robot.vert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        ((DcMotorEx) robot.frontLeft).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        ((DcMotorEx) robot.frontRight).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        ((DcMotorEx) robot.backLeft).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        ((DcMotorEx) robot.backRight).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));

        DcMotor FL = robot.frontLeft, FR = robot.frontRight, BL = robot.backLeft, BR = robot.backRight;
        int FLpos = 0, FRpos = 0, BLpos = 0, BRpos = 0;
        robot.vert.setPower(0);
        robot.raiseLift(0);
        robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.IntakeClose();
        robot.yaw1.setPosition(0.65);
        robot.yaw2.setPosition(0.05);
        robot.setMode(1);
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                        k = k+1;
                        telemetry.addData("k", k);
                        telemetry.update();
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }





        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        telemetry.update();
        //waitforStart();

        double gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*1;
        double lastpos = FL.getCurrentPosition();
        robot.imu1.resetYaw();

        sleep(500);
        robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.raiseLift(600);

        double Move1=1.7; // Need TUNE
        X=1300;



        robot.setMode(1);
        robot.setMode(3);

        sleep(200);







        while (BL.getCurrentPosition() > (-1100)){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            telemetry.addData("BL1",BL.getCurrentPosition());
        }

        robot.setMotorPowers(0);
            /*
            sleep(1000);
            robot.yaw1.setPosition(0.4);
            robot.yaw2.setPosition(0.3);
            sleep(500);
            robot.claw1.setPosition(0.8);
            robot.claw2.setPosition(1);
            sleep(500);
            */
        robot.IntakeOpen();
        sleep(500);
        robot.raiseLift(1200);
        robot.setMode(2);

        robot.moveRight(850, 0.5, 0.5, 0.5, 0.5);

        lastpos = FL.getCurrentPosition();

        // walk to centre
        while (FL.getCurrentPosition() < (lastpos + 650)){
            telemetry.addData("FL", FL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.update();
        }
        robot.setMotorPowers(0);
        telemetry.addData("imu", robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

        sleep(200);
            /*
            if (robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < -1){
                robot.rotateLeft(300, 0.1);
                while (robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < 0){
                    telemetry.addData("imu", robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    telemetry.update();
                }
            }
            else{
                robot.rotateRight(300, 0.1);
                while (Math.abs(robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 1){
                    telemetry.addData("imu", robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    telemetry.update();
                }
            }*/
        //double gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)/ 360;


        robot.setMode(1);
        robot.setMode(3);
        robot.raiseLift(430);


        while (BL.getCurrentPosition() < 1100){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;

            robot.setMotorPowers((0.5-gyrocal),(0.5+gyrocal),(0.5-gyrocal),(0.5+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        robot.setMotorPowers(0);



        robot.IntakeClose();
        sleep(500);
        robot.raiseLift(1000);
        sleep(800);
        robot.setMode(2);
        robot.setMode(1);
        robot.walkBackward(2500, 0.5);



        while (FL.getCurrentPosition() > -2300){
            //if (FL.getCurrentPosition() < (lastpos-1600)) robot.setMotorPowers(0.4);
            //else if (FL.getCurrentPosition() < (lastpos-2000)) robot.setMotorPowers(0.3);
            telemetry.addData("FL2", FL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.update();
        }
        robot.yaw1.setPosition(0.65);
        robot.yaw2.setPosition(0.05);
        robot.setMotorPowers(0);
        robot.raiseLift(4000);
        robot.setMode(1);
        robot.setMode(3);


        while (BL.getCurrentPosition() > (-800)){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            telemetry.addData("BL1",BL.getCurrentPosition());
        }
        robot.setMotorPowers(0);
        robot.setMode(1);
        robot.setMode(3);
        while (BL.getCurrentPosition() < 220){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((0.3-gyrocal),(0.3+gyrocal),(0.3-gyrocal),(0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("BLfw",BL.getCurrentPosition());

            telemetry.update();
        }

        robot.setMotorPowers(0);
        while (robot.vert.getCurrentPosition() < 2000){}
        sleep(300);
        robot.IntakeOpen();
        sleep(300);

        robot.setMode(1);
        robot.setMode(3);
        while (BL.getCurrentPosition() > -220){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((-0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(-0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("BLfw",BL.getCurrentPosition());

            telemetry.update();
        }
        robot.setMode(1);
        while (BL.getCurrentPosition() < 700){
            robot.setMode(3);
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((-0.3-gyrocal),(0.3+gyrocal),(0.3-gyrocal),(-0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            telemetry.addData("BL1",BL.getCurrentPosition());
        }

        robot.setMotorPowers(0);
        robot.setMode(1);
        robot.setMode(3);
        robot.raiseLift(300);
        while (BL.getCurrentPosition() < 2600){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((0.5-gyrocal),(0.5+gyrocal),(0.5-gyrocal),(0.5+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("BLfw",BL.getCurrentPosition());

            telemetry.update();
        }
        robot.setMotorPowers(0);

        //second cycle

        robot.IntakeClose();
        sleep(500);
        robot.raiseLift(1000);
        sleep(800);
        robot.setMode(2);
        robot.setMode(1);
        robot.walkBackward(2500, 0.5);



        while (FL.getCurrentPosition() > -2300){
            //if (FL.getCurrentPosition() < (lastpos-1600)) robot.setMotorPowers(0.4);
            //else if (FL.getCurrentPosition() < (lastpos-2000)) robot.setMotorPowers(0.3);
            telemetry.addData("FL2", FL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.update();
        }
        robot.yaw1.setPosition(0.65);
        robot.yaw2.setPosition(0.05);
        robot.setMotorPowers(0);
        robot.raiseLift(4000);
        robot.setMode(1);
        robot.setMode(3);


        while (BL.getCurrentPosition() > (-800)){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            telemetry.addData("BL1",BL.getCurrentPosition());
        }
        robot.setMotorPowers(0);
        robot.setMode(1);
        robot.setMode(3);
        while (BL.getCurrentPosition() < 220){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((0.3-gyrocal),(0.3+gyrocal),(0.3-gyrocal),(0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("BLfw",BL.getCurrentPosition());

            telemetry.update();
        }

        robot.setMotorPowers(0);
        while (robot.vert.getCurrentPosition() < 2000){}
        sleep(300);
        robot.IntakeOpen();
        sleep(300);

        robot.setMode(1);
        robot.setMode(3);
        while (BL.getCurrentPosition() > -220){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((-0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(-0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("BLfw",BL.getCurrentPosition());

            telemetry.update();
        }
        robot.setMode(1);
        while (BL.getCurrentPosition() < 700){
            robot.setMode(3);
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((-0.3-gyrocal),(0.3+gyrocal),(0.3-gyrocal),(-0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            telemetry.addData("BL1",BL.getCurrentPosition());
        }

        robot.setMotorPowers(0);
        robot.setMode(1);
        robot.setMode(3);
        robot.raiseLift(300);
        while (BL.getCurrentPosition() < 2600){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((0.5-gyrocal),(0.5+gyrocal),(0.5-gyrocal),(0.5+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("BLfw",BL.getCurrentPosition());

            telemetry.update();
        }
        robot.setMotorPowers(0);

        robot.walkBackward(2500, 0.5);

        while (FL.getCurrentPosition() > -2300){
            //if (FL.getCurrentPosition() < (lastpos-1600)) robot.setMotorPowers(0.4);
            //else if (FL.getCurrentPosition() < (lastpos-2000)) robot.setMotorPowers(0.3);
            telemetry.addData("FL2", FL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.update();
        }
        robot.yaw1.setPosition(0.65);
        robot.yaw2.setPosition(0.05);
        robot.setMotorPowers(0);
        robot.raiseLift(4000);
        robot.setMode(1);
        robot.setMode(3);


        while (BL.getCurrentPosition() > (-800)){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            telemetry.addData("BL1",BL.getCurrentPosition());
        }
        robot.setMotorPowers(0);
        robot.setMode(1);
        robot.setMode(3);
        while (BL.getCurrentPosition() < 220){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((0.3-gyrocal),(0.3+gyrocal),(0.3-gyrocal),(0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("BLfw",BL.getCurrentPosition());

            telemetry.update();
        }

        robot.setMotorPowers(0);
        while (robot.vert.getCurrentPosition() < 2000){}
        sleep(300);
        robot.IntakeOpen();
        sleep(300);

        robot.setMode(1);
        robot.setMode(3);
        while (BL.getCurrentPosition() > -220){
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((-0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(-0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("BLfw",BL.getCurrentPosition());

            telemetry.update();
        }
        robot.setMode(1);
        while (BL.getCurrentPosition() < 1600){
            robot.setMode(3);
            gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
            robot.setMotorPowers((-0.3-gyrocal),(0.3+gyrocal),(0.3-gyrocal),(-0.3+gyrocal));
            telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            telemetry.addData("BL1",BL.getCurrentPosition());
        }

        robot.setMotorPowers(0);
        robot.setMode(1);

        if(tagOfInterest == null){
            //default path
        }else{
            switch(tagOfInterest.id){
                case 1:
                    robot.setMode(1);
                    robot.setMode(3);
                    robot.raiseLift(300);
                    while (BL.getCurrentPosition() < 3000){
                        gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                        robot.setMotorPowers((0.5-gyrocal),(0.5+gyrocal),(0.5-gyrocal),(0.5+gyrocal));
                        telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                        telemetry.addData("BLfw",BL.getCurrentPosition());
                        telemetry.update();
                    }
                    robot.setMotorPowers(0);
                case 2:
                    robot.setMode(1);
                    robot.setMode(3);
                    robot.raiseLift(300);
                    while (BL.getCurrentPosition() < 2000){
                        gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                        robot.setMotorPowers((0.5-gyrocal),(0.5+gyrocal),(0.5-gyrocal),(0.5+gyrocal));
                        telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                        telemetry.addData("BLfw",BL.getCurrentPosition());
                        telemetry.update();
                    }
                    robot.setMotorPowers(0);
                case 3:
                    robot.setMotorPowers(0);
            }
            sleep(50000);
        }



    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
