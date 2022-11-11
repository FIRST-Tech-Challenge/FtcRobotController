package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * Write a detailed description of the autoClass
 */
@Autonomous(name = "FTC Wires Autonomous", group = "00-Autonomous" , preselectTeleOp = "FTC Wires TeleOp")
public class AutoOpMode extends LinearOpMode{

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

        while (!isStopRequested()) {
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            vision.runVuforiaTensorFlow();
            telemetry.clearAll();
            telemetry.addData("Start FTC Wires (ftcwires.org) Autonomous Mode adopted for Team","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Vision identified Parking Location", vision.identifiedparkingLocation);
            telemetry.update();

            //Game Play Button  is pressed
            if (opModeIsActive() && !isStopRequested()) {
                //Build Autonomous trajectory to be used based on starting position selected
                buildAuto();
                driveTrain.getLocalizer().setPoseEstimate(initPose);
                //Stop Vision process
                vision.deactivateVuforiaTensorFlow();
                runAuto();
                break;
            }
        }
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryAuto ;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d pickConePose;
    Pose2d dropConePose;
    Pose2d parkPose;

    public void buildAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-54, 36, Math.toRadians(0)); //Starting pose
                midWayPose = new Pose2d(-12, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(-12, 55, Math.toRadians(90)); //Choose the pose to move to the stack of cones
                dropConePose = new Pose2d(-12, 12, Math.toRadians(225)); //Choose the pose to move to the stack of cones
                switch(vision.identifiedparkingLocation){
                    case 1: parkPose = new Pose2d(-12, 60, Math.toRadians(180)); break; // Location 1
                    case 2: parkPose = new Pose2d(-12, 36, Math.toRadians(180)); break; // Location 2
                    case 3: parkPose = new Pose2d(-12, 11, Math.toRadians(180)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-54, -36, Math.toRadians(0));//Starting pose
                midWayPose = new Pose2d(-12, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(-12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                dropConePose = new Pose2d(-12, -12, Math.toRadians(135)); //Choose the pose to move to the stack of cones
                switch(vision.identifiedparkingLocation){
                    case 1: parkPose = new Pose2d(-12, -11, Math.toRadians(180)); break; // Location 1
                    case 2: parkPose = new Pose2d(-12, -36, Math.toRadians(180)); break; // Location 2
                    case 3: parkPose = new Pose2d(-12, -60, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_LEFT:
                initPose = new Pose2d(54, -36, Math.toRadians(180));//Starting pose
                midWayPose = new Pose2d(12, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                dropConePose = new Pose2d(12, -12, Math.toRadians(45)); //Choose the pose to move to the stack of cones
                switch(vision.identifiedparkingLocation){
                    case 1: parkPose = new Pose2d(12, -60, Math.toRadians(0)); break; // Location 1
                    case 2: parkPose = new Pose2d(12, -36, Math.toRadians(0)); break; // Location 2
                    case 3: parkPose = new Pose2d(12, -11, Math.toRadians(0)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                initPose = new Pose2d(54, 36, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(12, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(12, 55, Math.toRadians(90)); //Choose the pose to move to the stack of cones
                dropConePose = new Pose2d(12, 12, Math.toRadians(315)); //Choose the pose to move to the stack of cones
                switch(vision.identifiedparkingLocation){
                    case 1: parkPose = new Pose2d(12, 11, Math.toRadians(0)); break; // Location 1
                    case 2: parkPose = new Pose2d(12, 36, Math.toRadians(0)); break; // Location 2
                    case 3: parkPose = new Pose2d(12, 60, Math.toRadians(0)); break; // Location 3
                }
                break;
        }

        //Drop Preloaded Cone, Pick 5 cones and park
        trajectoryAuto = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(dropConePose)
                .addDisplacementMarker(() -> {
                    dropCone(); //Drop preloaded Cone
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(pickConePose)
                .addDisplacementMarker(() -> {
                    pickCone1();
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(dropConePose)
                .addDisplacementMarker(() -> {
                    dropCone();
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(pickConePose)
                .addDisplacementMarker(() -> {
                    pickCone2();
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(dropConePose)
                .addDisplacementMarker(() -> {
                    dropCone();
                })
                .addDisplacementMarker(() -> {
                    parkingComplete();
                })
                .lineToLinearHeading(parkPose)
                .build();
    }

    public void runAuto(){
        telemetry.setAutoClear(false);
        telemetry.addData("Running FTC Wires (ftcwires.org) Autonomous Mode adopted for Team:","TEAM NUMBER");
        telemetry.addData("---------------------------------------","");
        //Write any other actions to take during auto, or any other conditions for maneuvering
        driveTrain.followTrajectorySequence(trajectoryAuto);
    }

    //Write a method which is able to pick the cone depending on your subsystems
    public void pickCone1() {
        /*TODO: Add code to pick Cone 1 from stack*/
        telemetry.addData("Picked Cone: Stack", "1");
        telemetry.update();
    }
    public void pickCone2(){
        /*TODO: Add code to pick Cone 2 from stack*/
        telemetry.addData("Picked Cone: Stack", "2");
        telemetry.update();
    }
    //Write a method which is able to drop the cone depending on your subsystems
    int dropConeCounter = 0;
    public void dropCone(){
        /*TODO: Add code to drop cone on junction*/
        if (dropConeCounter == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", dropConeCounter);
        }
        telemetry.update();
        dropConeCounter += 1;
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
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous Mode adopted for Team:","TEAM NUMBER");
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

