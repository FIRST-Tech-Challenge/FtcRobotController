package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.team.auto.CSBaseLIO;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.team10515.GridSystem;


/*
 * This {@code class} acts as the driver-controlled program with Assistance for FTC team 10515 for the PowerPlay
 * challenge. By extending {@code PPRobot}, we already have access to all the robot subsystems,
 * so only tele-operated controls need to be defined here.
 *
 * The controls for this robot are:
 *  User 1:
 *      Drive:
 *          Left & Right joysticks     -> Mecanum drive
 *          Left-stick-button          -> Robot speed to default value
 *          Right-stick-button         -> Disable/enable movement
 *          Left-Trigger               -> Decrease robot speed
 *          Right-Trigger              -> Increase robot speed
 *      Lift:
 *          Dpad-up                    -> High
 *          Dpad-down                  -> Ground / Intake
 *          Dpad-right                 -> Medium
 *          Dpad-left                  -> Low
 *      Claw:
 *          Left-Bumper                -> Open
 *          Right-Bumper               -> Close
 *      Arm:
 *          B-Button (pressed)         -> Left
 *          X-Button (pressed)         -> Right
 *          A-Button (pressed)         -> Middle
 *          Y-Button (pressed)         -> Middle
 *  User 2:
 *      Drive:
 *          Left bumper (pressed)      ->
 *          Right bumper (pressed)     ->
 *      Lift:
 *          Left-trigger               ->
 *          Right-trigger              ->
 *          A-button (pressed)         ->
 *          Y-button (pressed)         ->
 *      Arm:
 *          Dpad-right                 ->
 *          Dpad-down                  ->
 *          Dpad-left                  ->
 *          Dpad-up                    ->
 *
 * @see UltimateGoalRobot
 */

@TeleOp(name = "PP TeleOp Assist", group = "Main")
@Disabled
public class PPTeleopAssistLAC extends CSTeleopRobotLIO {

    int[] blank = {};
    int[] pole01 = {0, 1, 0, 1, 0};
    int[] pole123 = {1, 2, 3, 2, 1};
    int[] pole03 = {0, 3, 0, 3, 0};
    int[][][] grid = {
            {blank, pole01, {0}},
            {pole01, pole123, {1}},
            {pole123, pole03, {2}},
            {pole03, pole123, {3}},
            {pole123, pole01, {4}},
            {pole01, blank, {5}}
    };

    public double currentTime = 0; // keep track of current time
    private boolean dropperLeft = true;
    private boolean stopintake = true;
    private boolean liftdown = true;
    private boolean freightloaded = false;
//    public double previousTime = 0; // keep track of last time A was pressed (Flicker was moved)
//    public double flickerInterval = 1; // after 1 second has passed since pressing A, move Flicker back to original position

//    public boolean shooterIsOn = false;
//    public ShooterStateMachine.State currentShooterSpeed = ShooterStateMachine.State.SPEED1;

//    public boolean allowMovement = true;
    private CSVP cv;
    private float confidence = 0;
    private String label = "";
    public double xSpeedMult = 1;
    public double ySpeedMult = 0.6;
    public double headingSpeedMult = 0.65;
    public Pose2d poseEstimate;
    public Vector2D currentPoseVector;



    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    public boolean spikeFromButton = false;
    public ElapsedTime timeSinceIntakeButton = new ElapsedTime();
    public double current = 0.0;
    public double lastCurrent = current;

    Telemetry.Item patternName;
    Telemetry.Item display;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    Mode currentMode = Mode.DRIVER_CONTROL;

    Pose2d allianceRedHubPosition = new Pose2d(-12,-40, Math.toRadians(0));
    //for now while roadrunner does not work well
    int x = 0;
    int y = 0;
    @Override
    public void init(){
        drive = new CSBaseLIO(hardwareMap, true);
        drive.setPoseEstimate(PoseStorage.currentPose);
    //    blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        //cv = new PPCV();
        //cv.initVuforia();
        //cv.initTfod(hardwareMap);
        super.init();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        super.loop();
        drive.update();
        lastCurrent = current;

//        //LED Code
//        if (spikeFromButton){
//            if (timeSinceIntakeButton.milliseconds() > 1000){
//                spikeFromButton = false;
//            }
//        }
//        if (current - lastCurrent > 125 && !freightloaded && !spikeFromButton) {
//            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
//            blinkinLedDriver.setPattern(pattern);
//            freightloaded = true;
//    //        stopintake = true;
//        }
//        else if(!freightloaded){
//            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
//            blinkinLedDriver.setPattern(pattern);
//        }
//        telemetry.addData("Freight? :", freightloaded);

        poseEstimate = drive.getPoseEstimate();
//        double clippedX = Range.clip(poseEstimate.getX(), -66.0, 66.0);
//        double clippedY = Range.clip(poseEstimate.getY(), -66.0, 66.0);
////        Pose2d clippedPose = new Pose2d(clippedX, clippedY, poseEstimate.getHeading());
////        drive.setPoseEstimate(clippedPose);
////        poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("Pose X: ", poseEstimate.getX());
//        telemetry.addData("Pose Y: ", poseEstimate.getY());
//        telemetry.addData("Clipped X: ", clippedX);
//        telemetry.addData("CLipped Y: ", clippedY);
////        double clippedY = Range.clip(poseEstimate.getY(), -66.0, 66.0);
//        //Speed calculations based on distance from alliance hubs
////        telemetry.addData("Position X: ", poseEstimate.getX());
////        telemetry.addData("Position Y: ", poseEstimate.getY());
//        currentPoseVector = new Vector2D(poseEstimate.getX(), poseEstimate.getY());
//        double blueHubDist = Vector2D.distance(currentPoseVector, blueHubVector);
//        double redHubDist = Vector2D.distance(currentPoseVector, redHubVector);
//        double calculatedXSpeed = xSpeedMult;
//        double calculatedYSpeed = ySpeedMult;
//        double calculatedHeadingSpeed = headingSpeedMult;
//        telemetry.addData("Distance Blue: ", blueHubDist);
//        telemetry.addData("Distance Red: ", redHubDist);
//        if (blueHubDist < 36 || redHubDist < 36){
//            double distanceProportionSquared = Math.pow((blueHubDist < redHubDist ? blueHubDist : redHubDist) / 36.0, 2);
//            calculatedXSpeed = xSpeedMult * distanceProportionSquared;
//            calculatedYSpeed = ySpeedMult * distanceProportionSquared;
//            calculatedHeadingSpeed = headingSpeedMult * distanceProportionSquared;
//        }
//        if (!slowToHubs || blueHubDist < 9 || redHubDist < 9){
//            calculatedXSpeed = xSpeedMult;
//            calculatedYSpeed = ySpeedMult;
//            calculatedHeadingSpeed = headingSpeedMult;
//        }
//        double slowFactor = 3.3;
//        double calculatedXSpeed = slowToHubs ? xSpeedMult/slowFactor : xSpeedMult;
//        double calculatedYSpeed = slowToHubs ? ySpeedMult/slowFactor : ySpeedMult;
//        double calculatedHeadingSpeed = slowToHubs ? headingSpeedMult/slowFactor : headingSpeedMult;
//        telemetry.addData("Speed X: ", calculatedXSpeed);
//        telemetry.addData("Speed Y: ", calculatedYSpeed);
//        telemetry.addData("Speed Heading: ", calculatedHeadingSpeed);


        switch (currentMode) {
            case DRIVER_CONTROL:
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
                telemetry.addData("X: ", gamepad1.left_stick_x);
                telemetry.addData("Y: ", gamepad1.left_stick_y);
                if(getEnhancedGamepad1().isaJustPressed()){

                }
/*
                if (getEnhancedGamepad1().isDpadLeftJustPressed()) {
                    // If the A button is pressed on gamepad1, we generate a splineTo()
                    // trajectory on the fly and follow it
                    // We switch the state to AUTOMATIC_CONTROL

                    TrajectorySequence allianceRedHub = drive.trajectorySequenceBuilder(poseEstimate)
                            .splineTo(new Vector2d(allianceRedHubPosition.getX(), allianceRedHubPosition.getY()), allianceRedHubPosition.getHeading())
                            .build();

                    drive.followTrajectorySequenceAsync(allianceRedHub);
                    currentMode = Mode.AUTOMATIC_CONTROL;
                }
                break;
 */
            case AUTOMATIC_CONTROL:
                // If x is pressed, we break out of the automatic following
                if (getEnhancedGamepad1().isyJustPressed()) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }

                if(getEnhancedGamepad1().isDpadUpJustPressed()){
                    TrajectorySequence liftTest = drive.trajectorySequenceBuilder(poseEstimate)
                            .forward(12)
                            .build();

                    drive.followTrajectorySequenceAsync(liftTest);
                }
                if(getEnhancedGamepad1().isDpadDownJustPressed()){
                    TrajectorySequence liftTest2 = drive.trajectorySequenceBuilder(poseEstimate)
                            .back(12)
                            .build();

                    drive.followTrajectorySequenceAsync(liftTest2);
                }
                // If drive finishes its task, cede control to the driver
                if (!drive.isBusy()) {
                    currentMode = Mode.DRIVER_CONTROL;
                }
                break;

        }

        int poleType = findNearestPole(new Vector2d());

        if (poleType == 0){
            //drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.EXTEND); //not true yet
        }
        else if(poleType == 1){

        }
        else if(poleType == 2){

        }
        else if(poleType == 3){

        }


        //telemetry.addData("left x: ", gamepad1.left_stick_x);
        //telemetry.addData("left y: ", gamepad1.left_stick_y);

        //-----------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 1

//        telemetry.addData("Gripper State: ", drive.robot.getCappingGripperSubsystem().getStateMachine().getState());
//        telemetry.addData("Arm State: ", drive.robot.getCappingArmSubsystem().getStateMachine().getState());
//        if(getEnhancedGamepad1().isDpadDownJustPressed()){
//            drive.robot.getCappingArmSubsystem().getStateMachine().updateState(CappingArmStateMachine.State.REST);
//            drive.robot.getCappingGripperSubsystem().getStateMachine().updateState(CappingGripperStateMachine.State.CLOSED);
//            telemetry.addLine("Pad1 Dpad Down");
//        }
//        if(getEnhancedGamepad1().isLeftBumperJustPressed()){
//            telemetry.addLine("Pad1 Left Bumper");
//            switch (drive.robot.getCappingArmSubsystem().getStateMachine().getState()) {
//                case REST:
//                    drive.robot.getCappingArmSubsystem().getStateMachine().updateState(CappingArmStateMachine.State.PICKUP_READY);
//                    drive.robot.getCappingGripperSubsystem().getStateMachine().updateState(CappingGripperStateMachine.State.OPEN);
//                    break;
//                case PICKUP_READY:
//                    drive.robot.getCappingArmSubsystem().getStateMachine().updateState(CappingArmStateMachine.State.TOP);
//                    break;
//                case TOP:
//                    drive.robot.getCappingArmSubsystem().getStateMachine().updateState(CappingArmStateMachine.State.RELEASE);
//                    break;
//                case RELEASE:
//                    drive.robot.getCappingArmSubsystem().getStateMachine().updateState(CappingArmStateMachine.State.REST);
//                    break;
//            }
//        }
//        if(getEnhancedGamepad1().isaJustPressed()){
//            telemetry.addLine("Pad1 A");
//            switch (drive.robot.getCappingGripperSubsystem().getStateMachine().getState()) {
//                case OPEN:
//                    drive.robot.getCappingGripperSubsystem().getStateMachine().updateState(CappingGripperStateMachine.State.GRIPPED);
//                    break;
//                case GRIPPED:
//                    drive.robot.getCappingGripperSubsystem().getStateMachine().updateState(CappingGripperStateMachine.State.OPEN);
//                    break;
//            }
//        }
//        if (getEnhancedGamepad1().isRightBumperJustPressed()){ //TODO: Change to gamepad 2?
//            slowToHubs = !slowToHubs;
//            telemetry.addData("Slow to hubs? :", slowToHubs);
//        }

        //-----------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 2


//
//        if(getEnhancedGamepad2().isaJustPressed()){
//            drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.RETRACT);
//            liftdown = true;
//            stopintake = false;
////            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.INTAKE);
//            telemetry.addLine("a pressed lift up: " + drive.robot.getElevSubsystem().getStateMachine().getState());
//        }
//
//        if(getEnhancedGamepad2().isyJustPressed()){
////            if (drive.robot.getCappingArmSubsystem().getStateMachine().getState() == CappingArmStateMachine.State.TOP) {
//                drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.EXTENDTOP);
//                stopintake = true;
//                liftdown = false;
////            }
//
//            telemetry.addLine("y pressed lift down: " + drive.robot.getElevSubsystem().getStateMachine().getState());
//        }

        currentTime = getRuntime();

    }
    public int findNearestPole(Vector2d robotPos){
        //Three things we need to know:
        //Which lane are we in (0-5)?
        //Are we closer to the poles on the left side of the lane or the right side of the lane?
        //Based on the above info and our y-coord, which pole are we closest to?

        robotPos.getX(); // 60 -> -60 TO 0 -> 5
        robotPos.getY(); // 60 -> 60 TO 0 -> 5
        double laneNumber = ((-robotPos.getX() + 60.0) * 5.0 / 120.0); //(-robotPos.getX() + 60) * (5 - 0) / (60 + 60) + 0
        int roundedLaneNumber = (int)Math.round(laneNumber);

        int closerToLeft;
        //do not necessarily need this for now
        if ((int)laneNumber == roundedLaneNumber){ //E.g. laneNumber is 2.3, then we are closer to the left side of lane 2
            closerToLeft = 1;
        }
        else {
            closerToLeft = 0;
        }

        double yLevel = ((-robotPos.getY() + 60.0) * 10.0 / 120.0); //(value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
        int roundedYLevel = (int)Math.round(yLevel); //Odd numbers are seams, even numbers are midpoints of tiles

        int[] typeOfPoleRow = grid[roundedLaneNumber][closerToLeft]; //Can be blank, pole01, pole123, or pole03
        if (typeOfPoleRow == blank){
            return -1; //No pole near us
        }
        if (roundedYLevel % 2 == 1) {
            int index = (roundedYLevel - 1)/2; //Since rounded level is odd, subtracting 1 and dividing by 2 will be a number between 0 and 4
            int typeOfPole = typeOfPoleRow[index];
            return typeOfPole;
        }
        //If rounded level is even (robot at midpoint of tile), what should we do?
        //We can either say there is no pole near us or go to the height of the highest pole out of the four around us
        return -1; //No pole near us (robot at midpoint or logic failed)
    }
}
