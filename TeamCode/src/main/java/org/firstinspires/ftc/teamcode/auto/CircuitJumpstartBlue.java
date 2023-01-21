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

@Autonomous(name="BlueAlliance Near BlueTerminal")
public class CircuitJumpstartBlue extends BaseOpMode {
    Hardware2 robot = new Hardware2(true);
    // Handle hardware stuff...

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    SleeveDetection detector = new SleeveDetection();
    OpenCvCamera phoneCam;


//        ABOVE_GROUND(300),
//        MEDIUM_JUNCTION(1850),
//        DOWN_OVER_JUNCTION(-600),
//        FROM_OVER_JUNCTION_TO_STARTING_HEIGHT(-1550),
//        TO_TOP_CONE_STACK(170),
//        SMALL_JUNCTION(1150);


    public void terminalScoring() {
        DriveUtils.encoderClaw(this,0.4,300,5);
        DriveUtils.rotate(90,0.3,this);
        AutoUtils.tileMove(this, "Front", 1,0.4);
        DriveUtils.moveClaw(this, "Open");
        AutoUtils.tileMove(this, "Back", 1,0.4);
        AutoUtils.tileStrafe(this, "Left" ,2,0.4);
    }
    public void grabbingCone() {
        AutoUtils.tileMove(this, "Front", 1.15,0.3);
        DriveUtils.encoderClaw(this,0.4,170,5);
        DriveUtils.moveClaw(this, "Close");
        sleep(1000);
        DriveUtils.encoderClaw(this, 0.4, 600,5);
        AutoUtils.tileMove(this, "Back", 1,0.3);
        DriveUtils.encoderClaw(this,0.4,-770,5);
    }
    public void smallJunctionScoring(){
        AutoUtils.tileStrafe(this, "Right" ,0.5,0.4);
        DriveUtils.encoderClaw(this,0.6,1150,5); // Raises the claw up to match the height of the junction
        DriveUtils.encoderDrive(this,0.3,-5.3,-5.3,5); // Drives forward so the cone will drop on the junction
        DriveUtils.encoderClaw(this,-0.5,-600,5); //Lowers the claw onto the junction
        DriveUtils.moveClaw(this,"Open"); // Opens the claw
        DriveUtils.encoderDrive(this,0.3,5.3,5.3,5); //Drives Back
        AutoUtils.tileStrafe(this, "Left" ,0.5,0.4);
    }


    @Override
    public void runOpMode() {
        robot.initTeleOpIMU(hardwareMap);

        // robot logic...

        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera
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
        DriveUtils.moveClaw(this,"Open");
        waitForStart();
        //...

            String position = getPosition();
            position = "center";
            if (position == "center") {
                terminalScoring();
                grabbingCone();
                smallJunctionScoring();
//                DriveUtils.encoderStrafe(this,0.4,13.5,5);
                // Drives forward to go into the middle parking area
                //DriveUtils.encoderDrive(this, 0.5, 10, 10, 7);
            } else if (position == "right") {
                terminalScoring();
                grabbingCone();
                smallJunctionScoring();
//                DriveUtils.encoderStrafe(this,0.4,40.5,5);
            } else if (position == "left") {
                terminalScoring();
                grabbingCone();
                smallJunctionScoring();
//                DriveUtils.encoderStrafe(this,0.4,-13.5,5);
            }


        // more robot logic...
    }

    @Override
    public Hardware2 getRobot() {
        return robot;
    }
}