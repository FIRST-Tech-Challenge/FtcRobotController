package org.firstinspires.ftc.teamcode.OpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * Write a detailed description of the autoClass
 */
@Autonomous(name = "Autonomous", group = "00-Autonomous" , preselectTeleOp = "TeleOp")
public class AutoOpMode extends LinearOpMode{

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public Vision vision = new Vision();
    public DriveTrain driveTrain = new DriveTrain(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        /*Create your Subsystem Objects*/

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();

        // Initiate Camera on Init.
        vision.activateVuforiaTensorFlow();

        while (!isStopRequested()) {
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            vision.runVuforiaTensorFlow();
            telemetry.clearAll();
            telemetry.addData("Vision target Detected : ", vision.identifiedTarget);
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
    TrajectorySequence trajectoryAuto;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d pickConePose;
    Pose2d dropConePose;
    Pose2d parkPose;

    public void buildAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-54, 36, Math.toRadians(0)); //Starting pos when on blue alliance
                midWayPose = new Pose2d(-12, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(-12, 55, Math.toRadians(90)); //Choose the pose to move to the stack of cones
                dropConePose = new Pose2d(-12, 12, Math.toRadians(225)); //Choose the pose to move to the stack of cones
                switch(vision.identifiedTarget){
                    case 1: parkPose = new Pose2d(-12, 60, Math.toRadians(180)); // Location 1
                    case 2: parkPose = new Pose2d(-12, 36, Math.toRadians(180)); break; // Location 2
                    case 3: parkPose = new Pose2d(-12, 12, Math.toRadians(180)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-54, -36, Math.toRadians(0));//Starting pos when on blue alliance
                midWayPose = new Pose2d(-12, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(-12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                dropConePose = new Pose2d(-12, -12, Math.toRadians(135)); //Choose the pose to move to the stack of cones
                switch(vision.identifiedTarget){
                    case 1: parkPose = new Pose2d(-12, -12, Math.toRadians(180)); break; // Location 1
                    case 2: parkPose = new Pose2d(-12, -36, Math.toRadians(180)); break; // Location 2
                    case 3: parkPose = new Pose2d(-12, -60, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_LEFT:
                initPose = new Pose2d(54, -36, Math.toRadians(180));//Starting pos when on blue alliance
                midWayPose = new Pose2d(12, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                dropConePose = new Pose2d(12, -12, Math.toRadians(45)); //Choose the pose to move to the stack of cones
                switch(vision.identifiedTarget){
                    case 1: parkPose = new Pose2d(12, -60, Math.toRadians(0)); break; // Location 1
                    case 2: parkPose = new Pose2d(12, -36, Math.toRadians(0)); break; // Location 2
                    case 3: parkPose = new Pose2d(12, -12, Math.toRadians(0)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                initPose = new Pose2d(54, 36, Math.toRadians(180)); //Starting pos when on blue alliance
                midWayPose = new Pose2d(12, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(12, 55, Math.toRadians(90)); //Choose the pose to move to the stack of cones
                dropConePose = new Pose2d(12, 12, Math.toRadians(315)); //Choose the pose to move to the stack of cones
                switch(vision.identifiedTarget){
                    case 1: parkPose = new Pose2d(12, 12, Math.toRadians(0)); break; // Location 1
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
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(pickConePose)
                .addDisplacementMarker(() -> {
                    pickCone3();
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(dropConePose)
                .addDisplacementMarker(() -> {
                    dropCone();
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(pickConePose)
                .addDisplacementMarker(() -> {
                    pickCone4();
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(dropConePose)
                .addDisplacementMarker(() -> {
                    dropCone();
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(pickConePose)
                .addDisplacementMarker(() -> {
                    pickCone5();
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(dropConePose)
                .addDisplacementMarker(() -> {
                    dropCone();
                })
                .lineToLinearHeading(parkPose)
                .build();
    }

    /**
     * Safe method to wait so that stop button is also not missed
     * @param time time in ms to wait
     */
    public void safeWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            driveTrain.update();
        }
    }


    public void runAuto(){
        //Write any other actions to take during auto, or any other conditions for maneuvering
        driveTrain.followTrajectorySequence(trajectoryAuto);
    }

    //Write a method which is able to pick the cone depending on your subsystems
    public void pickCone1(){/*TODO: Add code to pick Cone 1 from stack*/}
    public void pickCone2(){/*TODO: Add code to pick Cone 2 from stack*/}
    public void pickCone3(){/*TODO: Add code to pick Cone 3 from stack*/}
    public void pickCone4(){/*TODO: Add code to pick Cone 4 from stack*/}
    public void pickCone5(){/*TODO: Add code to pick Cone 5 from stack*/}
    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(){/*TODO: Add code to drop Cone on pole*/}

    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("Blue Left: (X)", "");
            telemetry.addData("Blue Right: (Y)", "");
            telemetry.addData("Red Left: (B)", "");
            telemetry.addData("Red Right: (A)", "");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                telemetry.addData("Start Position: ", startPosition);
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                telemetry.addData("Start Position: ", startPosition);
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                telemetry.addData("Start Position: ", startPosition);
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                telemetry.addData("Start Position: ", startPosition);
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
        telemetry.addData("StartPose : ", startPosition);
        telemetry.update();
    }
}

