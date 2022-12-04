package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
 */
@Autonomous(name = "FTC Wires Auto Only Park", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class AutoOnlyPark extends LinearOpMode{

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public Vision vision;
    public DriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        /*Create your Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        vision = new Vision(hardwareMap);

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();

        // Initiate Camera on Init.
        vision.activateVuforiaTensorFlow();

        while (!isStopRequested() && !opModeIsActive()) {
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            vision.runVuforiaTensorFlow();
            telemetry.clearAll();
            telemetry.addData("Start FTC Wires (ftcwires.org) Autonomous Mode adopted for Team","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Vision identified Parking Location", vision.identifiedparkingLocation);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Stop Vision process
            vision.deactivateVuforiaTensorFlow();

            //Build parking trajectory based on last detected target by vision
            buildParking();
            driveTrain.getLocalizer().setPoseEstimate(initPose);

            //run Autonomous trajectory
            runParking();
        }

        //Trajectory is completed, display Parking complete
        parkingComplete();
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryParking ;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d parkPose;

    //Build parking trajectory based on target detected by vision
    public void buildParking(){
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-54, 36, Math.toRadians(0)); //Starting pose
                midWayPose = new Pose2d(-12, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                switch(vision.identifiedparkingLocation){
                    case 1: parkPose = new Pose2d(-12, 60, Math.toRadians(0)); break; // Location 1
                    case 2: parkPose = new Pose2d(-12, 36, Math.toRadians(0)); break; // Location 2
                    case 3: parkPose = new Pose2d(-12, 11, Math.toRadians(0)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-54, -36, Math.toRadians(0));//Starting pose
                midWayPose = new Pose2d(-12, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                switch(vision.identifiedparkingLocation){
                    case 1: parkPose = new Pose2d(-12, -11, Math.toRadians(0)); break; // Location 1
                    case 2: parkPose = new Pose2d(-12, -36, Math.toRadians(0)); break; // Location 2
                    case 3: parkPose = new Pose2d(-12, -60, Math.toRadians(0)); break; // Location 3
                }
                break;
            case RED_LEFT:
                initPose = new Pose2d(54, -36, Math.toRadians(180));//Starting pose
                midWayPose = new Pose2d(12, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                switch(vision.identifiedparkingLocation){
                    case 1: parkPose = new Pose2d(12, -60, Math.toRadians(180)); break; // Location 1
                    case 2: parkPose = new Pose2d(12, -36, Math.toRadians(180)); break; // Location 2
                    case 3: parkPose = new Pose2d(12, -11, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                initPose = new Pose2d(54, 36, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(12, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                switch(vision.identifiedparkingLocation){
                    case 1: parkPose = new Pose2d(12, 11, Math.toRadians(180)); break; // Location 1
                    case 2: parkPose = new Pose2d(12, 36, Math.toRadians(180)); break; // Location 2
                    case 3: parkPose = new Pose2d(12, 60, Math.toRadians(180)); break; // Location 3
                }
                break;
        }
        
        trajectoryParking = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(parkPose)
                .build();
    }

    //Run Auto trajectory and parking trajectory
    public void runParking(){
        telemetry.setAutoClear(false);
        telemetry.addData("Running FTC Wires (ftcwires.org) Autonomous Mode adopted for Team:","TEAM NUMBER");
        telemetry.addData("---------------------------------------","");
        telemetry.update();
        //Run the trajectory built for Auto and Parking
        driveTrain.followTrajectorySequence(trajectoryParking);
    }

    public void parkingComplete(){
        telemetry.addData("Parked in Location", vision.identifiedparkingLocation);
        telemetry.update();
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) AutoOnlyPark adopted for Team:","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }
}

