package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxBL_X;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxBL_Y;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxHeight;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getCenterX;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getHighestX;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getHighestY;
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

    //The unit here is pixels
    int targetX;
    double errorX;
    double dividerX = 500;
    double alignPowerAdded;
    int slidePosition = 0;
    int targetHeight = 0;
    double imuHeading = 0;
    double leftX;
    double leftY;
    double rightX;
    int counter;



    //The unit here is boxes
    int targetDistance;

    @Override
    public void runOpMode(){

//        initMotorsBlue();
//
//        calibrateNavXIMU();

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

        targetX = getXResolution()/2; //<-- this SHOULD be the resolution at level1 (check-able)
        level1Aligned = false;
        level2Aligned = false;
        level3Aligned = false;
//        isIMURecorded = false;
        visionAutoActivated = false;

//        slidePosition = motorSlide.getCurrentPosition();

        counter = 0;
        while(opModeIsActive()){


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
//                stopMotors();
//                motorSlide.setPower(0);
            }

            //this means we are no longer looking at our object of choice
            if(getLargestSize() == 0) {
                levelCounter = 1;
                level1Aligned = false;
                level2Aligned = false;
                level3Aligned = false;
                visionAutoActivated = false;
//                stopMotors();
//                motorSlide.setPower(0);
            }

            errorX = targetX - getCenterX();


            dividerX = 400;

            alignPowerAdded = errorX/dividerX;

            if(Math.abs(alignPowerAdded) > 0.17){
                alignPowerAdded = (errorX/(Math.abs(errorX)))*0.17;
            }


            // Uncomment to supposedly turn on Teleop,
//            leftY = -gamepad1.left_stick_y;
//            leftX = gamepad1.left_stick_x;
//            rightX = gamepad1.right_stick_x;
//
//                motorFL.setPower(leftY + leftX + rightX);
//                motorBL.setPower(leftY - leftX + rightX);
//                motorFR.setPower(leftY - leftX - rightX);
//                motorBR.setPower(leftY + leftX - rightX);
//

            // This is for timing iterations
                System.out.println("-----------------------------------------------------------------------------Counter: " + counter);
                counter++;




            if(visionAutoActivated){



                if(levelCounter == 1 && Math.abs(errorX) < 32){//TODO will need to add distance condition
                    level1Aligned = true;
                    levelCounter = 2;
                    telemetry.addLine("Going into level2!...");
                    telemetry.addLine("errorX: " + errorX);
                    telemetry.addLine("errorX divide thingy: " + (errorX/(Math.abs(errorX))));

                    telemetry.update();
//                    stopMotors();
                    sleep(500);
                    //Robot is in front of pole well enough, entering level2...
                }

                if(levelCounter == 1 && level1Aligned == false){
//                    dividerX = 300; //<-- Arbitrary number for tuning
//            motorFL.setPower(-alignPowerAdded);
//            motorBL.setPower(-alignPowerAdded);
//            motorFR.setPower(alignPowerAdded);
//            motorBR.setPower(alignPowerAdded);
                }

                if(levelCounter == 2 && Math.abs(errorX) < 5){ //TODO will need to add distance condition
                    level2Aligned = true;
//                    imuHeading = getCumulativeZ(); //Need
//                    isIMURecorded = true; // honestly wholly redundant
                    levelCounter = 3;
//                    stopMotors();
                    telemetry.addLine("Going into level3!...");
                    telemetry.update();
                    sleep(500);
                }


                if(levelCounter == 2 && level2Aligned == false){ //TODO feed different inputs into this to make more aggressive
//
//
                    if(Math.abs((errorX/230)) < 0.14){
                        alignPowerAdded = (double)errorX/230;
                    }else{
                        alignPowerAdded =(errorX/(Math.abs(errorX)))*0.14;
                    }
//                    telemetry.addLine("alignPowerAdded: " + alignPowerAdded);
//                    telemetry.addLine("errorX: " + errorX);
//                    telemetry.update();
//                    sleep(10000);

//                    }

//                     motorFL.setPower(-alignPowerAdded);
//                     motorBL.setPower(-alignPowerAdded);
//                     motorFR.setPower(alignPowerAdded);
//                     motorBR.setPower(alignPowerAdded);
                }

                if (levelCounter == 3 && getPercentColor() < 20){
                    level3Aligned = true;

                }

                if(levelCounter == 3 && level3Aligned == false){
//                    motorSlide.setPower(0.5);
//                    slidePosition = motorSlide.getCurrentPosition();

                    //Slide go up <-- Honestly just use a consistent power for ease
                }

                //For all the marbles, this is the sequence that stacks
                if(level3Aligned == true){
//                    slidePosition = motorSlide.getCurrentPosition();
//                    stopMotors();
//                    telemetry.addLine("We going to the top babeeeeeeee");
//                    telemetry.addLine("Slide position: " + slidePosition);
//                    telemetry.update();
//                    sleep(3000);
                    if(slidePosition >= 0 && slidePosition <= lowHeight){
                        targetHeight = lowHeight;
                    }else if(slidePosition > lowHeight && slidePosition <= midHeight){
                        targetHeight = midHeight;
                    }else if(slidePosition > midHeight && slidePosition <= highHeight){
                        targetHeight = highHeight;
                    }

//                    clawClamp();
//                    GoToHeight(targetHeight);
//                    sleep(250);
//                    driveForDistance(0.1,Direction.FORWARD,0.2,imuHeading);
//                    sleep(250);
//                    GoToHeight(targetHeight - 30);
//                    sleep(100);
//                    clawRelease();
//                    sleep(100);
//                    GoToHeight(targetHeight);
//                    sleep(100);
//                    driveForDistance(0.1, Direction.BACKWARD, 0.2, imuHeading);
//                    goToDown();

                    levelCounter = 1;
                    level1Aligned = false;
                    level2Aligned = false;
                    level3Aligned = false;
                    visionAutoActivated = false;
                    targetX = 320; //TODO Avoid hard coding this value? Or maybe just take from the original resolution setting above
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
//            telemetry.addLine("LowestX: " + getLowestX());
//            telemetry.addLine("HighestX: " + getHighestX());
//            telemetry.addLine("LowestY: " + getLowestY());
//            telemetry.addLine("HighestY: " + getHighestY());
            telemetry.addLine("targetX: " + targetX);
            telemetry.addLine("centerX: " + getCenterX());
            telemetry.addLine("Power Applied: " + alignPowerAdded);
//            telemetry.addLine("level1Aligned?: " + level1Aligned);
//            telemetry.addLine("level2Aligned?: " + level2Aligned);
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
