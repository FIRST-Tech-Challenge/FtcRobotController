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

package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.robot.TurtleRobotAuto;
import java.util.ArrayList;

@Autonomous(name="AUTONOMOUS TAP HERE")
public class AUTONOMOUS extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    TurtleRobotAuto robot = new TurtleRobotAuto(this);

    static final double FEET_PER_METER = 3.28084;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   =  3.7795276;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private ElapsedTime runtime = new ElapsedTime();
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
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
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

        telemetry.setMsTransmissionInterval(50);

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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
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

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            strafeLeft(0.15, 1000);
            stopRobot();
            sleep(1000);
//            straight(0.15, 900);
//            stopRobot();
//            stop();
//            EncoderDrive(robot, 0.45, -25, -25, 25,-25, 2);
            EncoderDrive(robot, 0.45, 35, 35,35, 35, 2);
        }else if(tagOfInterest.id == MIDDLE){
//            straight(0.15, 1000);
//            stopRobot();
//            stop();
            EncoderDrive(robot, 0.5, 35, 35,35, 35, 2);
        }else{
            strafeRight(0.15, 1100);
            stopRobot();
            sleep(1000);
//            straight(0.15, 900);
//            stopRobot();
//            stop();
//            EncoderDrive(robot, 0.45, 25, -25, -25,25, 2);
            EncoderDrive(robot, 0.45, 35, 35,35, 35, 2);
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
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
    public void straight(double power, int time) {
        robot.leftfrontmotor.setPower(power);
        robot.leftbackmotor.setPower(power);
        robot.rightfrontmotor.setPower(power);
        robot.rightbackmotor.setPower(power);
        sleep(time);
    }
    public void strafeRight(double power, int time) {
        robot.leftfrontmotor.setPower(power);
        robot.leftbackmotor.setPower(-power);
        robot.rightfrontmotor.setPower(-power);
        robot.rightbackmotor.setPower(power);
        sleep(time);
    }
    public void strafeLeft(double power, int time) {
        robot.leftfrontmotor.setPower(-power);
        robot.leftbackmotor.setPower(power);
        robot. rightfrontmotor.setPower(power);
        robot.rightbackmotor.setPower(-power);
        sleep(time);
    }
    public void stopRobot() {
        robot.leftfrontmotor.setPower(0);
        robot.leftbackmotor.setPower(0);
        robot.rightfrontmotor.setPower(0);
        robot.rightbackmotor.setPower(0);
    }
    public void left(double power, int time) {
        robot.leftfrontmotor.setPower(power);
        robot.leftbackmotor.setPower(power);
        robot. rightfrontmotor.setPower(-power);
        robot.rightbackmotor.setPower(-power);
        sleep(time);
    }
    public void right(double power, int time) {
        robot.leftfrontmotor.setPower(-power);
        robot.leftbackmotor.setPower(-power);
        robot. rightfrontmotor.setPower(power);
        robot.rightbackmotor.setPower(power);
        sleep(time);
    }
    public void EncoderDrive(TurtleRobotAuto turtleRobotAuto, double speed,
                             double leftfrontInches, double leftbackInches,
                             double rightfrontInches, double rightbackInches,
                             double timeoutS) {
        int newLeftfrontTarget;
        int newLeftbackTarget;
        int newRightfrontTarget;
        int newRightbackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftfrontTarget = robot.leftfrontmotor.getCurrentPosition() + (int) (leftfrontInches * COUNTS_PER_INCH);
            newLeftbackTarget = robot.leftbackmotor.getCurrentPosition() + (int) (leftbackInches * COUNTS_PER_INCH);
            newRightfrontTarget = robot.rightfrontmotor.getCurrentPosition() + (int) (rightfrontInches * COUNTS_PER_INCH);
            newRightbackTarget = robot.rightbackmotor.getCurrentPosition() + (int) (rightbackInches * COUNTS_PER_INCH);
            robot.leftfrontmotor.setTargetPosition(newLeftfrontTarget);
            robot.leftbackmotor.setTargetPosition(newLeftfrontTarget);
            robot.rightfrontmotor.setTargetPosition(newRightfrontTarget);
            robot.rightbackmotor.setTargetPosition(newRightbackTarget);


            // Turn On RUN_TO_POSITION
            robot.leftfrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftbackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightfrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightbackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftfrontmotor.setPower(Math.abs(speed));
            robot.leftbackmotor.setPower(Math.abs(speed));
            robot.rightfrontmotor.setPower(Math.abs(speed));
            robot.rightbackmotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the  will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the  continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    && (robot.leftfrontmotor.isBusy() &&
                    robot.leftbackmotor.isBusy()
                    && robot.rightfrontmotor.isBusy()
                    && robot.rightbackmotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d",
                        newLeftfrontTarget,
                        newLeftbackTarget,
                        newRightfrontTarget,
                        newRightbackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftfrontmotor.getCurrentPosition(),
                        robot.leftbackmotor.getCurrentPosition(),
                        robot.rightfrontmotor.getCurrentPosition(),
                        robot.rightbackmotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftfrontmotor.setPower(0);
            robot.leftbackmotor.setPower(0);
            robot.rightfrontmotor.setPower(0);
            robot.rightbackmotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}