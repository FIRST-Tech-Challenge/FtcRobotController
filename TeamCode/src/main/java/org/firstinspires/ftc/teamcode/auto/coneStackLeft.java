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

@Autonomous(name="Cone Stack Left")
public class coneStackLeft extends BaseOpMode {
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
        DriveUtils.encoderStrafe(this,0.4,23.5,5);//strafes to the right and is ready to go forward
        AutoUtils.tileMove(this,"Front",2,0.5);//goes forward toward junction
        AutoUtils.tileStrafe(this, "Left" ,0.440,0.4);// strafes left to be inline with the junction
        DriveUtils.encoderClaw(this,0.7,2650,10); //brings claw up over junction
        DriveUtils.encoderDrive(this,0.5,-5.5,-5.5,5); // Drives forward so the cone will drop on the junction
        DriveUtils.encoderClaw(this,0.7, -300,5);//claw goes down so cone drops
        sleep(100);
        DriveUtils.moveClaw(this, "Open");
        sleep(100);
        DriveUtils.encoderDrive(this,0.5,5.57,5.57,5); //Drives Back
        DriveUtils.encoderClaw(this,0.7,-2350,10); //brings claw back down
        AutoUtils.tileStrafe(this, "Left" ,0.42,0.5);//strafes left closer to stack
        DriveUtils.rotate(83,0.3,this);//rotates toward stack
    }
    public void grabbingCone() {
        AutoUtils.tileMove(this, "Front", 0.85,0.5);//moves forward to stack of cones
        DriveUtils.encoderClaw(this,0.7,240,5);//brings claw up to be inline with stack
        DriveUtils.encoderDrive(this,0.3,-5.65,-5.65,5);//drives to the stack
        DriveUtils.moveClaw(this, "Close");//closes the claw into the cone
        sleep(500);//sleeps for half a second so the cone doesnt fall
        DriveUtils.encoderClaw(this, 0.7, 495,5);//picking up cone off cone stack
        AutoUtils.tileMove(this, "Back", 0.87,0.4);//moves back to score onto small junction
        DriveUtils.encoderDrive(this,0.3,6,6,5);//moves back more precisely
    }
    public void smallJunctionScoring(){
        AutoUtils.tileStrafe(this, "Left" ,0.48,0.4);//strafes to be in line with the small junction
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
        Vision Initialization
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
        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();

        String position = getPosition();
        if (position == "center") {
            highJunctionScoring();
            grabbingCone();
            smallJunctionScoring();
            //it ends in the center so we wont have to move to park
        } else if (position == "right") {
            highJunctionScoring();
            grabbingCone();
            smallJunctionScoring();
            AutoUtils.tileStrafe(this,"Right", .43, 0.6);//moves right
            AutoUtils.tileMove(this,"Back", .9, 0.6);//moves back into the right parking station
        } else if (position == "left") {
            highJunctionScoring();
            grabbingCone();
            smallJunctionScoring();
            AutoUtils.tileStrafe(this,"Left", .72, 0.6);//moves left
            AutoUtils.tileStrafe(this,"Right", .2, 0.6);//moves right back in line with the parking position b/c we moved more to avoid the signal

            AutoUtils.tileMove(this,"Front", 1, 0.6); // moves into parking position
        }


        // more robot logic...
    }

    @Override
    public Hardware2 getRobot() {
        return robot;
    }
}