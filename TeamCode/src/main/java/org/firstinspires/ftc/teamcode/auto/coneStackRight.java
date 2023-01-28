package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.config.DriveUtils.getPosition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.AutoUtils;
import org.firstinspires.ftc.teamcode.config.BaseOpMode;
import org.firstinspires.ftc.teamcode.config.DriveUtils;
import org.firstinspires.ftc.teamcode.config.Hardware2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Cone Stack Right")
public class coneStackRight extends BaseOpMode {
    Hardware2 robot = new Hardware2(true);
    // Handle hardware stuff...

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    SleeveDetection detector = new SleeveDetection();
    OpenCvCamera phoneCam;


//        ABOVE_GROUND(300),
//        MEDIUM_JUNCTION(2000),
//        DOWN_OVER_JUNCTION(-600),
//        FROM_OVER_JUNCTION_TO_STARTING_HEIGHT(-1700),
//        TO_TOP_CONE_STACK(170),
//        SMALL_JUNCTION(250),
//        HIGH_JUNCTION(2650);

    public void highJunctionScoring() {
        DriveUtils.encoderClaw(this,0.7,300,5); // Raises the claw to BASE height
        DriveUtils.encoderStrafe(this ,0.4,-23.5,5); // strafes to the left to avoid the signal
        AutoUtils.tileMove(this,"Front",2,0.5); // Move forward to high junction
        AutoUtils.tileStrafe(this, "Right" ,0.525,0.4); // Strafes to be in front of the high junction
        DriveUtils.encoderClaw(this,0.7,2650,10); // raises claw up above the junction
        DriveUtils.encoderDrive(this,0.5,-5,-5,5); // Drives forward so the cone will drop on the junction
        DriveUtils.encoderClaw(this,0.7, -300,5); // Drops down over junction
        sleep(100);
        DriveUtils.moveClaw(this, "Open"); // lets cone go
        sleep(100);
        DriveUtils.encoderDrive(this,0.5,4,4,5); //Drives Back
        DriveUtils.encoderClaw(this,0.7,-2350,10); // put claw back to base position
        AutoUtils.tileStrafe(this, "Right" ,0.40,0.5); // strafes closer to cone stack
        DriveUtils.rotate(-87,0.3,this); // turns towards the cone stack
    }
    public void grabbingCone() {
        AutoUtils.tileMove(this, "Front", 0.85,0.5); // Moves towards the cone stack
        DriveUtils.encoderClaw(this,0.7,240,5); // raises to the height of the  top cone
        DriveUtils.encoderDrive(this,0.3,-6,-6,5); //  drives forward
        DriveUtils.moveClaw(this, "Close"); // closes on the cone
        sleep(500);
        DriveUtils.encoderClaw(this, 0.7, 590,5); // lifts the cone up
        AutoUtils.tileMove(this, "Back", 0.85,0.4); // moves back to go to the small junction
        DriveUtils.encoderDrive(this,0.3,6,6,5); // moves back to go to the small junction
    }
    public void smallJunctionScoring(){
        AutoUtils.tileStrafe(this, "Right" ,0.55,0.4);//  strafes to be in front of the small junction
        DriveUtils.encoderClaw(this,0.7,320,5); // Raises the claw up to match the height of the junction
        DriveUtils.encoderDrive(this,0.5,-6.75,-6.75,5); // Drives forward so the cone will drop on the junction
        DriveUtils.encoderClaw(this,0.7,-300,5); //Lowers the claw onto the junction
        DriveUtils.moveClaw(this,"Open"); // Opens the claw
        DriveUtils.encoderDrive(this,0.4,6.75,6.75,5); //Drives Back

    }


    @Override
    public void runOpMode() {
        robot.initTeleOpIMU(hardwareMap);

        // robot logic...

        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera

        /*
        Vision Code Below
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        // Connect to the camera
        phoneCam.openCameraDevice();
        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        phoneCam.setPipeline(detector);
        // Remember to change the camera rotation
        phoneCam.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT);



        DriveUtils.initiate(this);
        DriveUtils.moveClaw(this,"Close");
        waitForStart();
        //...

            String position = getPosition();
            if (position == "center") {
                highJunctionScoring();
                grabbingCone();
                smallJunctionScoring();
            } else if (position == "right") {
                highJunctionScoring();
                grabbingCone();
                smallJunctionScoring();
                AutoUtils.tileStrafe(this,"Right", .7, 0.6);
                AutoUtils.tileStrafe(this, "Left", .2, 0.6); // All the parking stuff from the small junction to position 3
                AutoUtils.tileMove(this,"Front", 1, 0.6);
            } else if (position == "left") {
                highJunctionScoring();
                grabbingCone();
                smallJunctionScoring();
                AutoUtils.tileStrafe(this,"Left", .45, 0.6);
                AutoUtils.tileMove(this,"Back", 1, 0.6); // All the parking stuff from the small junction to position 1
            }


        // more robot logic...
    }

    @Override
    public Hardware2 getRobot() {
        return robot;
    }
}