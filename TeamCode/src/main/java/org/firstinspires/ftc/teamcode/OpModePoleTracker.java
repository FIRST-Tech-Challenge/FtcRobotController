package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxBL_X;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxBL_Y;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxHeight;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getCenterX;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getHighestX;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getHighestY;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLargestObjectWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLargestSize;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel1Assigment;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel2Assigment;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel2Capable;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevelString;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLowestX;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLowestY;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getMinRectHeight;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getMinRectWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getPercentColor;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getRectHeight;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getRectWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getXResolution;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getYResolution;

import static org.firstinspires.ftc.teamcode.Variables.*;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="PoleTracker", group="A")
public class OpModePoleTracker extends DriveMethods {
    String level = "one";
    int levelCounter = 1;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;



    //The unit here is pixels
    int targetX;
    int targetWidth;
    double errorX;
    int errorWidth;
    double dividerX = 500;
    double alignPowerAddedX;
    double alignPowerAddedWidth;
    int slidePosition = 0;
    int targetHeight = 0;
    double imuHeading = 0;
    double leftX;
    double leftY;
    double rightX;


    //The unit here is boxes
    int targetDistance;

    @Override
    public void runOpMode(){

        initMotorsBlue();
            blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

            pattern = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
            blinkinLedDriver.setPattern(pattern);


            calibrateNavXIMU();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        PipePoleTracker pipePoleTracker = new PipePoleTracker(level);
        camera.setPipeline(pipePoleTracker);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //Adjust this to reduce load?????
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);


            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        waitForStart();

        targetX = 225; //<-- this SHOULD be the resolution at level1 (check-able)
        targetWidth = 15;
        level1Aligned = false;
        level2Aligned = false;
        level3Aligned = false;
//        isIMURecorded = false;
        visionAutoActivated = false;

        GoToHeight(collectHeight);
        slidePosition = motorSlide.getCurrentPosition();


        while(opModeIsActive()){

            leftY = -gamepad1.left_stick_y;
            leftX = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;

            motorFL.setPower((leftY + leftX + rightX)/2);
            motorBL.setPower((leftY - leftX + rightX)/2);
            motorFR.setPower((leftY - leftX - rightX)/2);
            motorBR.setPower((leftY + leftX - rightX)/2);

            if(level2Capable == false && visionAutoActivated == false){
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                blinkinLedDriver.setPattern(pattern);
            }
            if(level2Capable == true && visionAutoActivated == false){
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                blinkinLedDriver.setPattern(pattern);
            }
            if(visionAutoActivated && levelCounter == 1 || levelCounter == 2){
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                blinkinLedDriver.setPattern(pattern);
            }

            if(visionAutoActivated && levelCounter == 3){
                pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
                blinkinLedDriver.setPattern(pattern);
            }

            //Button triggers
            if(gamepad2.a && getLevel2Capable()){
//                levelCounter = 2;
                visionAutoActivated = true;


            }

            if(gamepad2.x){
                levelCounter = 1;
                level1Aligned = false;
                level2Aligned = false;
                level3Aligned = false;
                visionAutoActivated = false;
                stopMotors();
                motorSlide.setPower(0);
            }

            //this means we are no longer looking at our object of choice
            if(getLargestSize() == 0) {
                levelCounter = 1;
                level1Aligned = false;
                level2Aligned = false;
                level3Aligned = false;
                visionAutoActivated = false;
                stopMotors();
                motorSlide.setPower(0);
            }

            errorX = targetX - getCenterX();
            errorWidth = targetWidth - getLargestObjectWidth();

            dividerX = 400;






            if(visionAutoActivated){

                alignPowerAddedX = errorX/dividerX;

                alignPowerAddedWidth = (double)errorWidth/45;


                if(Math.abs(alignPowerAddedX) > 0.14){
                    alignPowerAddedX = (errorX/(Math.abs(errorX)))*0.14;
                }

                if(levelCounter == 1 && Math.abs(errorX) < 32){//TODO will need to add distance condition
                    level1Aligned = true;
                    imuHeading = getCumulativeZ(); //Need
                    levelCounter = 2;
                    telemetry.addLine("level1 complete!");
                    telemetry.addLine("errorX: " + errorX);
                    telemetry.addLine("errorX divide thingy: " + (errorX/(Math.abs(errorX))));

                    telemetry.update();
                    stopMotors();
                    //Robot is in front of pole well enough, entering level2...
                }

                if(levelCounter == 1 && level1Aligned == false){

                motorFL.setPower(-alignPowerAddedX);
                motorBL.setPower(-alignPowerAddedX);
                motorFR.setPower(alignPowerAddedX);
                motorBR.setPower(alignPowerAddedX);

                }

                //Level2 below (untested at the moment - 1/17/23)
                if(levelCounter == 2 && Math.abs(errorWidth) <= 3){ //TODO will need to add distance condition
                    level2Aligned = true;
                    levelCounter = 3;
                    stopMotors();
                    telemetry.addLine("Level2 Complete!");
                    telemetry.addLine("Largest Oject Width: " + getLargestObjectWidth());
                    telemetry.addLine("Error width: "+ errorWidth);
                    telemetry.update();
                }


                if(levelCounter == 2 && level2Aligned == false){ //TODO feed different inputs into this to make more aggressive

                     motorFL.setPower(alignPowerAddedWidth - alignPowerAddedX/2.75);
                     motorBL.setPower(alignPowerAddedWidth - alignPowerAddedX/2.75);
                     motorFR.setPower(alignPowerAddedWidth + alignPowerAddedX/2.75);
                     motorBR.setPower(alignPowerAddedWidth + alignPowerAddedX/2.75);
                }

                if (levelCounter == 3 && getPercentColor() < 10 && getPercentColor() > 0){
                    level3Aligned = true;

                    telemetry.addLine("We're at the top of the pole!");
                    telemetry.addLine("level3Aligned: " + level3Aligned);
                    telemetry.addLine("Percent Color: " + getPercentColor());
                    telemetry.update();
                    sleep(1000);


                }

                if(levelCounter == 3 && level3Aligned == false){
                    clawClamp();
                    motorSlide.setPower(0.5);
                    slidePosition = motorSlide.getCurrentPosition();
                    telemetry.addLine("Measuring the pole height!");
                    telemetry.addLine("Percent Color: " + getPercentColor());
                    telemetry.update();

                    //Slide go up <-- Honestly just use a consistent power for ease
                }

                //For all the marbles, this is the sequence that stacks
                if(level3Aligned == true){
                    slidePosition = motorSlide.getCurrentPosition();
                    stopMotors();
                    telemetry.addLine("We going to the top babeeeeeeee");
                    telemetry.addLine("Slide position: " + slidePosition);
                    telemetry.addLine("targetHeight: " + targetHeight);
                    telemetry.update();
//                    sleep(500);
                    if(slidePosition >= 0 && slidePosition <= lowHeight){
                        targetHeight = lowHeight;
                    }else if(slidePosition > lowHeight && slidePosition <= midHeight){
                        targetHeight = midHeight;
                    }else if(slidePosition > midHeight && slidePosition <= highHeight){
                        targetHeight = highHeight;
                    }

                    clawClamp();
                    GoToHeight(targetHeight);
                    sleep(300);
                    driveForDistance(0.16,Direction.FORWARD,0.2,imuHeading);
//                    sleep(250);
                    GoToHeight(targetHeight - 75);
                    sleep(350);
                    clawRelease();
                    sleep(200);
                    GoToHeight(targetHeight);
                    sleep(300);
                    driveForDistance(0.16, Direction.BACKWARD, 0.2, imuHeading);
                    goToDown();

                    levelCounter = 1;
                    level1Aligned = false;
                    level2Aligned = false;
                    level3Aligned = false;
                    visionAutoActivated = false;
                    targetX = 225; //TODO Avoid hard coding this value? Or maybe just take from the original resolution setting above
//                    dividerX = 200;
                    //Back to manual driving!!!

                }
            }



//            if(level.equals("two") && Math.abs(errorX) < 8){
//                level2Aligned = true;
//            }



//            if(level2Aligned == false){
////                motorFL.setPower(alignPowerAdded);
////            motorBL.setPower(alignPowerAdded);
////            motorFR.setPower(-alignPowerAdded);
////            motorBR.setPower(-alignPowerAdded);
//
//            }




            if(levelCounter == 1){
                level = "one";
            }

            if(levelCounter == 2){
                level = "two";
            }

            if(levelCounter == 3){
                level = "three";
            }




            telemetry.addLine("errorX: " + errorX);
            telemetry.addLine("errorWidth: " + errorWidth);
            telemetry.addLine("current width: " + getLargestObjectWidth());
            telemetry.addLine("Current Level: " + getLevelString());
//            telemetry.addLine("Level1 Assigment: " + getLevel1Assigment());
//            telemetry.addLine("Level2 Assignment : " + getLevel2Assigment());
//            telemetry.addLine("X_resolution: " + getXResolution());
//            telemetry.addLine("Y_resolution: " + getYResolution());
            telemetry.addLine("Level 2 Capable?: " + getLevel2Capable());
            telemetry.addLine("Percent Color: " + getPercentColor());
//            telemetry.addLine("Focus Rectangle Width " + getRectWidth());
//            telemetry.addLine("Focus Rectangle Height: " + getRectHeight());
//            telemetry.addLine("Rectangle Min Width: " + getMinRectWidth());
//            telemetry.addLine("Rectangle Min Height: " + getMinRectHeight());
//            telemetry.addLine("Box Width: " + getBoxWidth());
//            telemetry.addLine("Box Height: " + getBoxHeight());
//            telemetry.addLine("Center X: " + getCenterX());
            telemetry.addLine("LowestX: " + getLowestX());
            telemetry.addLine("LowestY: " + getLowestY());
            telemetry.addLine("HighestX: " + getHighestX());
            telemetry.addLine("HighestY: " + getHighestY());
            telemetry.addLine("targetX: " + targetX);
            telemetry.addLine("centerX: " + getCenterX());
            telemetry.addLine("Power Applied X: " + alignPowerAddedX);
            telemetry.addLine("Power Applied Width: " + alignPowerAddedWidth);
//            telemetry.addLine("level1Aligned?: " + level1Aligned);
            telemetry.addLine("level3Aligned?: " + level3Aligned);
            telemetry.addLine("Activated?: " + visionAutoActivated);
//            telemetry.addLine("IMU Heading: " + imuHeading);
//            telemetry.addLine("Slide Position: " + slidePosition);
//            telemetry.addLine("Largest Size: " + getLargestSize());










            telemetry.update();

            pipePoleTracker = new PipePoleTracker(level);
            camera.setPipeline(pipePoleTracker);
        }
    }
}
