package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class AutoTestAprilTag extends LinearOpMode {
 //   SpikeCam.location mySpikeLocation;

    // This is a SHORT side Auton
    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
   //     CyDogsChassis.Direction parkingSpot = CyDogsChassis.Direction.LEFT;
   //     CyDogsChassis.Direction drivePath = CyDogsChassis.Direction.RIGHT;

        CyDogsSparky mySparky = new CyDogsSparky(this);
   //     mySparky.initializeSpikeCam(SpikeCam.TargetColor.RED);

        CyDogsAprilTags myReader = new CyDogsAprilTags(this);
        myReader.initAprilTag("Webcam 2");
        sleep(4000);
        mySparky.initializeDevices();
    //    mySparky.initializePositions();

        telemetry.addData("starting program","now");
        telemetry.update();

        AprilTagDetection myDetection = myReader.GetAprilTag(1);

        // Wait for the start button to be pressed on the driver station
        waitForStart();


        if (opModeIsActive()) {
            sleep(10000);

        }
    }
}



