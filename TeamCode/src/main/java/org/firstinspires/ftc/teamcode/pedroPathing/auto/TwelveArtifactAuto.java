//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.ManagerColorSensor;
//import org.firstinspires.ftc.teamcode.ManagerFlywheel;
//import org.firstinspires.ftc.teamcode.ManagerIntake;
//import org.firstinspires.ftc.teamcode.ManagerLimelight;
//import org.firstinspires.ftc.teamcode.ManagerResetRotator;
//import org.firstinspires.ftc.teamcode.ManagerRotator;
//import org.firstinspires.ftc.teamcode.ManagerStateMachine;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//
//@Autonomous(name = "Auto: 12 Artifacts", group = "Examples")
//public class TwelveArtifactAuto extends OpMode {
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//
//    // Where the robot is placed BEFORE a match starts
//    private final Pose startPoseRedWall = new Pose(88, 8, Math.toRadians(90));
//    private final Pose startPoseRedGoal = new Pose(118.638, 129.298, Math.toRadians(216));
//    private final Pose startPoseBlueWall = new Pose(56, 8, Math.toRadians(90));
//    private final Pose startPoseBlueGoal = new Pose(25.671, 129.795, Math.toRadians(-36));
//
//    // Where the robot can SHOOT from
//    private final Pose scorePoseRed = new Pose(97.104, 92.945, Math.toRadians(45));
//    private final Pose scorePoseBlue = new Pose(46.563, 92.945, Math.toRadians(135));
//
//    // Halfway point from wallStart to score
//    private final Pose midWayToScorePoseRed = new Pose(95.104, 90.945, Math.toRadians(90));
//    private final Pose midWayToScorePoseBlue = new Pose(50.563, 90.945, Math.toRadians(90));
//
//    // Where the robot goes to hit the gate
//    private final Pose gatePoseRed = new Pose(128.766, 73.889, Math.toRadians(90));
//    private final Pose gatePoseBlue = new Pose(16.084, 73.889, Math.toRadians(90));
//
//    // Where the robot can PICKUP from
//    private final Pose pickupPoseRed0 = new Pose(93.899, 83.869, Math.toRadians(180));
//    private final Pose pickupPoseRed1 = new Pose(93.899, 58.510, Math.toRadians(180));
//    private final Pose pickupPoseRed2 = new Pose(93.899, 33.482, Math.toRadians(180));
//    private final Pose pickupPoseBlue0 = new Pose(50.101, 83.869, Math.toRadians(0));
//    private final Pose pickupPoseBlue1 = new Pose(50.101, 58.510, Math.toRadians(0));
//    private final Pose pickupPoseBlue2 = new Pose(50.101, 33.482, Math.toRadians(0));
//
//    // Where the robot ENDS the match
//    private final Pose endPoseRed = new Pose(83.738, 40.101, Math.toRadians(0));
//    private final Pose endPoseBlue = new Pose(60.207, 40.101, Math.toRadians(180));
//
//
//    // Poses used to calculate paths
//    public Pose midWayToScorePose = midWayToScorePoseBlue;
//    public Pose startPose = startPoseBlueWall;
//    public Pose scorePose = scorePoseBlue;
//    public Pose pickupPose0 = pickupPoseBlue0;
//    public Pose pickupPose1 = pickupPoseBlue1;
//    public Pose pickupPose2 = pickupPoseBlue2;
//    public Pose gatePose = gatePoseBlue;
//    public Pose endPose = endPoseBlue;
//
//    private PathChain startToScorePath, scoreToEndPath;
//
//
//    // References to other classes
//    MecanumDrive drive;
//    ManagerStateMachine stateMachine;
//    ManagerRotator rotatorManager;
//    ManagerFlywheel flywheelManager;
//    ManagerColorSensor colorSensorManager;
//    ManagerIntake intakeManager;
//    ManagerResetRotator resetManager;
//    ManagerLimelight limelightManager;
//
//
//    // Information about the STARTING position
//    boolean isRedAlliance = false;
//    boolean isWallSide = true;
//    public Pose currentTargetPose = startPose;
//
//    // Path states
//    public int pathStateA, pathStateFlywheel, pathStateIntake, pathStatePivotArm = 0;
//    public int pathStateB, pathStateC = -1;
//
//    // Various delays
//    double flywheelDelay = 1; // 0.4
//    double pivotArmDelay = 1; // 0.4
//    double rotatorDelayB = 1; // 0.4
//    double rotatorDelayC = 1; // 0.6
//    double intakeTimerC = 1; // 1.5
//    double midWayPauseDelay = 1; //0.1
//
//
//    // Variables for State Machine A
//    int nextSetOfArtifactsA = 1; // 0 = highest, 1 = middle, 2 = lowest
//
//    // Variables for State Machine B
//    int shotsFiredB = 0;
//    boolean hasStartedRotatingB = false;
//
//    // Variables for State Machine C
//    int artifactsPickedUpC = 0;
//    double pickupMovementAmount = 5; // The sign is overwritten in buildPaths()
//    double pickupPrepInitialMovementAmount = 8;
//    boolean hasStartedRotatingC = false;
//    boolean hasMovedC = false;
//
//
//    public void buildPaths() {
//
//
//        // Determine what startPose to use
//        if (isRedAlliance) {
//            if (isWallSide) {
//                startPose = startPoseRedWall;
//            } else {
//                startPose = startPoseRedGoal;
//            }
//        } else {
//            if (isWallSide) {
//                startPose = startPoseBlueWall;
//            } else {
//                startPose = startPoseBlueGoal;
//            }
//        }
//
//
//        // Determine other variables based on alliance
//        if (isRedAlliance) {
//            midWayToScorePose = midWayToScorePoseRed;
//            scorePose = scorePoseRed;
//            pickupPose0 = pickupPoseRed0;
//            pickupPose1 = pickupPoseRed1;
//            pickupPose2 = pickupPoseRed2;
//            endPose = endPoseRed;
//            gatePose = gatePoseRed;
//
//            pickupMovementAmount = Math.abs(pickupMovementAmount); // Force it positive (Corresponding to going right)
//            pickupPrepInitialMovementAmount = Math.abs(pickupMovementAmount); // Force it positive (Corresponding to going right)
//        } else {
//            midWayToScorePose = midWayToScorePoseBlue;
//            scorePose = scorePoseBlue;
//            pickupPose0 = pickupPoseBlue0;
//            pickupPose1 = pickupPoseBlue1;
//            pickupPose2 = pickupPoseBlue2;
//            endPose = endPoseBlue;
//            gatePose = gatePoseBlue;
//
//            pickupMovementAmount = 0 - Math.abs(pickupMovementAmount); // Force it negative (Corresponding to going left)
//            pickupPrepInitialMovementAmount = 0 - Math.abs(pickupMovementAmount); // Force it negative (Corresponding to going left)
//        }
//
//        // Update the targetPose. This function is only called before Start() so the robot hasn't moved.
//        currentTargetPose = startPose;
//
//
//        if (isWallSide) {
//            startToScorePath = follower.pathBuilder()
//                    .addPath(new BezierLine(startPose, midWayToScorePose))
//                    .setLinearHeadingInterpolation(startPose.getHeading(), midWayToScorePose.getHeading())
//                    .build();
//        } else {
//            startToScorePath = follower.pathBuilder()
//                    .addPath(new BezierLine(startPose, scorePose))
//                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//                    .build();
//        }
//
//
//        // Update the final path chain
//        scoreToEndPath = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, endPose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
//                .build();
//
//    }
//
//
//
//
//
//
//    /**
//     * Primary 3 state machines
//     **/
//
//    public void autonomousPathUpdateA() {
//                    /* You could check for
//            - Follower State: "if(!follower.isBusy()) {}"
//            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//            - Robot Position: "if(follower.getPose().getX() > 36) {}"
//            */
//
//        switch (pathStateA) {
//            case -1:
//                break;
//            case 0:
//
//                // Moving to the firing position
//                if (!follower.isBusy()) {
//
//                    follower.followPath(startToScorePath, true);
//                }
//
//                if (pathTimer.getElapsedTimeSeconds() >= midWayPauseDelay) {
//                    setPathStateA(1);
//                }
//                break;
//
//            case 1:
//                // Moving to the firing position
//                if (!follower.isBusy()) {
//
//                    follower.followPath(makeGoToScorePath(), true);
//                    setPathStateA(2);
//                }
//                break;
//
//            case 2:
//
//                // Start flywheel & intake.
//                pathStateFlywheel = 1;
//                pathStateIntake = 1;
//
//                if (pathTimer.getElapsedTimeSeconds() >= flywheelDelay) {
//                    setPathStateA(3);
//                }
//                break;
//
//            case 3:
//
//                // Recursive firing
//                if ( pathStateB == -1 && !follower.isBusy() && follower.getPose().getY() > 90) {
//
//                    // Either start firing or continue to the next state, based on shotsFired
//                    if (shotsFiredB == 0) {
//                        setPathStateB(0);
//                    } else if (shotsFiredB >= 3) {
//                        setPathStateA(4);
//                    }
//
//                }
//                break;
//
//            case 4:
//
//                // Turn off the flywheel & intake. No need for a timer here
//                pathStateFlywheel = 0;
//                pathStateIntake = 0;
//                setPathStateA(5);
//                break;
//
//            case 5:
//
//                // Move to the next pickup position
//                if (!follower.isBusy()) {
//
//                    follower.followPath(makeScoreToPickupPath(nextSetOfArtifactsA), true);
//                    setPathStateA(6);
//                }
//                break;
//
//            case 6:
//
//                // Recursive pickup. Note that it cannot go to state 7
//                if ( pathStateC == -1 && !follower.isBusy() ) {
//
//                    // Either start firing or continue to the next state, based on shotsFired
//                    if (artifactsPickedUpC == 0) {
//                        setPathStateC(0);
//                    } else if (artifactsPickedUpC >= 3) {
//
//                        if (nextSetOfArtifactsA >= 3) {
//                            setPathStateA(7);
//                        } else {
//                            setPathStateA(1);
//                        }
//
//                        nextSetOfArtifactsA++;
//                        shotsFiredB = 0;
//                        artifactsPickedUpC = 0;
//                    }
//
//                }
//                break;
//
//            case 7:
//
//                // Turn off the intake & flywheel. No need for a timer here
//                pathStateIntake = 0;
//                pathStateFlywheel = 0;
//                setPathStateA(8);
//                break;
//
//            case 8:
//
//                // Move to the ending position
//                if (!follower.isBusy()) {
//
//                    follower.followPath(scoreToEndPath, true);
//                    currentTargetPose = endPose; // This line is not technically necessary at the moment
//                    setPathStateA(-1);
//                }
//                break;
//
//        }
//    }
//
//
//
//
//
//
//
//
//
//
//
//
//    public void autonomousPathUpdateB() {
//        switch (pathStateB) {
//            case -1:
//                break;
//            case 0:
//                // Snap to an initial firing position.
//
//                if (!hasStartedRotatingB) {
//                    hasStartedRotatingB = true;
//
//                    rotatorManager.currentSnappingPos = rotatorManager.getClosestFiringEncoderPos();
//                    rotatorManager.enter_RunToPosition((int) rotatorManager.currentSnappingPos, rotatorManager.inRotator_TargetPower_FIRING);
//                    rotatorManager.currentRotatorIndex = rotatorManager.getClosestFiringSlotIndex((int) rotatorManager.currentSnappingPos);
//                }
//
//
//                if (pathTimer.getElapsedTimeSeconds() >= rotatorDelayB) {
//                    hasStartedRotatingB = false;
//                    setPathStateB(1);
//                }
//                break;
//
//            case 1:
//
//                double multiplier = getMultiplier();
//
//                // Cycle rotator for motif
//                if (!hasStartedRotatingB) {
//                    hasStartedRotatingB = true;
//
//                    rotatorManager.currentSnappingPos -= rotatorManager.distanceBetweenSlots * multiplier;
//                    rotatorManager.setTargetPosition((int) rotatorManager.currentSnappingPos);
//                }
//
//
//                if (pathTimer.getElapsedTimeSeconds() >= rotatorDelayB) {
//                    hasStartedRotatingB = false;
//                    setPathStateB(2);
//                }
//                break;
//
//            case 2:
//
//                // Lower arm
//                pathStatePivotArm = 1;
//
//                if (pathTimer.getElapsedTimeSeconds() >= pivotArmDelay) {
//                    setPathStateB(3);
//                }
//                break;
//
//            case 3:
//                // Raise arm
//                pathStatePivotArm = 0;
//
//                if (pathTimer.getElapsedTimeSeconds() >= pivotArmDelay) {
//                    shotsFiredB++; // Increment count
//
//                    // Check if all 3 artifacts have been fired
//                    if (shotsFiredB >= 3) {
//                        setPathStateB(-1);
//                    } else {
//                        setPathStateB(4);
//                    }
//                }
//                break;
//
//            case 4:
//
//                // Cycle rotator :(
//                if (!hasStartedRotatingB) {
//                    hasStartedRotatingB = true;
//
//                    rotatorManager.currentSnappingPos -= rotatorManager.distanceBetweenSlots;
//                    rotatorManager.setTargetPosition((int) rotatorManager.currentSnappingPos);
//                }
//
//
//                if (pathTimer.getElapsedTimeSeconds() >= rotatorDelayB) {
//                    hasStartedRotatingB = false;
//                    setPathStateB(2);
//                }
//                break;
//
//        }
//    }
//
//
//
//
//
//
//
//
//
//
//    public void autonomousPathUpdateC() {
//        switch (pathStateC) {
//            case -1:
//                break;
//            case 0:
//                // Snap to an initial pickup position.
//
//                if (!hasStartedRotatingC) {
//                    hasStartedRotatingC = true;
//
//                    rotatorManager.currentSnappingPos = rotatorManager.getClosestPickupEncoderPos();
//                    rotatorManager.enter_RunToPosition((int) rotatorManager.currentSnappingPos, rotatorManager.inRotator_TargetPower_PICKUP);
//                    rotatorManager.currentRotatorIndex = rotatorManager.getClosestPickupSlotIndex((int) rotatorManager.currentSnappingPos);
//                }
//
//
//                if (pathTimer.getElapsedTimeSeconds() >= rotatorDelayC) {
//                    hasStartedRotatingC = false;
//                    setPathStateC(1);
//                }
//                break;
//
//            case 1:
//
//                // Move forwards slightly
//                if (!follower.isBusy() && !hasMovedC) {
//
//                    boolean movesExtra = false;
//                    if (artifactsPickedUpC == 0) movesExtra = true;
//
//                    follower.followPath(makeHorizontalPath(movesExtra), 0.6, true);
//                    hasMovedC = true;
//                }
//
//                if (pathTimer.getElapsedTimeSeconds() >= intakeTimerC) {
//                    hasMovedC = false; // Reset this for next iteration
//
//                    artifactsPickedUpC++;
//                    if (artifactsPickedUpC >= 3) {
//                        setPathStateC(-1);
//                    } else {
//                        setPathStateC(2);
//                    }
//                }
//                break;
//
//            case 2:
//
//                // Cycle rotator :(
//                if (!hasStartedRotatingC) {
//                    hasStartedRotatingC = true;
//
//                    rotatorManager.currentSnappingPos -= rotatorManager.distanceBetweenSlots;
//                    rotatorManager.setTargetPosition((int) rotatorManager.currentSnappingPos);
//                }
//
//
//                if (pathTimer.getElapsedTimeSeconds() >= rotatorDelayC) {
//                    hasStartedRotatingC = false;
//                    setPathStateC(1);
//                }
//                break;
//
//        }
//    }
//
//
//
//
//
//
//
//
//
//    /**
//     * State machines for individual functions
//     **/
//    public void autonomousFlywheelUpdate() {
//        switch (pathStateFlywheel) {
//            case -1:
//                break;
//            case 0:
//                flywheelManager.runFlywheel_Manual(false);
//                break;
//            case 1:
//                flywheelManager.runFlywheel_Manual(true);
//                break;
//        }
//    }
//
//
//
//
//
//
//
//
//
//
//
//    public void autonomousIntakeUpdate() {
//        switch (pathStateIntake) {
//            case -1:
//                break;
//            case 0:
//                intakeManager.runManualIntake(false, false, true);
//                break;
//            case 1:
//                intakeManager.runManualIntake(true, false, true);
//                break;
//        }
//    }
//
//
//
//
//
//
//
//    public void autonomousPivotArmUpdate() {
//        switch (pathStatePivotArm) {
//            case -1:
//                break;
//            case 0:
//                flywheelManager.runPivotArm_Manual(false);
//                break;
//            case 1:
//                flywheelManager.runPivotArm_Manual(true);
//                break;
//        }
//    }
//
//
//
//
//
//
//
//    /**
//     * These change the states of the paths and actions. It will also reset the timers of the individual switches
//     **/
//    public void setPathStateA(int pState) {
//        pathStateA = pState;
//        pathTimer.resetTimer();
//    }
//
//    public void setPathStateB(int pState) {
//        pathStateB = pState;
//        pathTimer.resetTimer();
//    }
//
//    public void setPathStateC(int pState) {
//        pathStateC = pState;
//        pathTimer.resetTimer();
//    }
//
//
//    /**
//     * Misc functions
//     **/
//    private PathChain makeHorizontalPath(boolean moveExtra) {
//        Pose targetPose = new Pose(currentTargetPose.getX() + pickupMovementAmount + (moveExtra ? pickupPrepInitialMovementAmount : 0), currentTargetPose.getY(), currentTargetPose.getHeading());
//        PathChain newPath = follower.pathBuilder()
//                .addPath(new BezierLine(currentTargetPose, targetPose))
//                .setLinearHeadingInterpolation(currentTargetPose.getHeading(), targetPose.getHeading())
//                .build();
//
//        currentTargetPose = targetPose;
//        return newPath;
//    }
//
//
//
//    private PathChain makeScoreToPickupPath(int index) {
//        Pose targetPose;
//        switch (index) {
//            case 1:
//                targetPose = pickupPose0;
//                break;
//            case 2:
//                targetPose = pickupPose1;
//                break;
//            case 3:
//                targetPose = pickupPose2;
//                break;
//            default:
//                targetPose = pickupPose0; // This line exists as a backup. It should NEVER run.
//                break;
//        }
//
//        PathChain newPath = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, targetPose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), targetPose.getHeading())
//                .build();
//
//        currentTargetPose = targetPose;
//        return newPath;
//    }
//
//
//
//    private PathChain makeGoToScorePath() {
//        PathChain newPath = follower.pathBuilder()
//                .addPath(new BezierLine(currentTargetPose, scorePose))
//                .setLinearHeadingInterpolation(currentTargetPose.getHeading(), scorePose.getHeading())
//                .build();
//
//        currentTargetPose = scorePose;
//        return newPath;
//    }
//
//
//
//
//    private double getMultiplier() {
//        switch (limelightManager.greenMotifPos) {
//            case -1:
//                return 0;
//            case 0:
//                switch (nextSetOfArtifactsA) {
//                    case 1:
//                        return 0;
//                    case 2:
//                        return 2;
//                    case 3:
//                        return 1;
//                    default:
//                        return 0;
//                }
//            case 1:
//                switch (nextSetOfArtifactsA) {
//                    case 1:
//                        return 2;
//                    case 2:
//                        return 1;
//                    case 3:
//                        return 0;
//                    default:
//                        return 0;
//                }
//            case 2:
//                switch (nextSetOfArtifactsA) {
//                    case 1:
//                        return 1;
//                    case 2:
//                        return 0;
//                    case 3:
//                        return 2;
//                    default:
//                        return 0;
//                }
//            default:
//                return 0;
//        }
//    }
//
//
//    /*private PathChain makeGoToGatePath() {
//        PathChain newPath = follower.pathBuilder()
//                .addPath(new BezierLine(currentTargetPose, gatePose))
//                .setLinearHeadingInterpolation(currentTargetPose.getHeading(), gatePose.getHeading())
//                .build();
//
//        currentTargetPose = gatePose;
//        return newPath;
//    }*/
//
//
//
//    /**
//     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
//     **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        autonomousPathUpdateA();
//        autonomousPathUpdateB();
//        autonomousPathUpdateC();
//
//        autonomousFlywheelUpdate();
//        autonomousIntakeUpdate();
//        autonomousPivotArmUpdate();
//        limelightManager.updateLimelightMotif();
////        limelightManager.moveCameraArm(Math.toDegrees(Math.atan2(72 - follower.getPose().getX(), 144 - follower.getPose().getY()) - 360));
//
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData(" ", " ");
//
//        telemetry.addData("pathStateA: ", pathStateA);
//        telemetry.addData("pathStateB: ", pathStateB);
//        telemetry.addData("pathStateC: ", pathStateC);
//        telemetry.addData("pathStateIntake: ", pathStateIntake);
//        telemetry.addData("pathStatePivotArm", pathStatePivotArm);
//        telemetry.addData("pathStateFlywheel", pathStateFlywheel);
//        telemetry.addData(" ", " ");
//
//        telemetry.addData("Shots Fired: ", shotsFiredB);
//        telemetry.addData("Next set of Artifacts", nextSetOfArtifactsA);
//        telemetry.addData("Has Rotated?: ", hasStartedRotatingB);
//        telemetry.addData("Timer: ", pathTimer.getElapsedTimeSeconds());
//        telemetry.addData(" ", " ");
//
//        telemetry.addData("Servo Pos: ", limelightManager.cameraArm.getPosition());
////        telemetry.addData("Angle: ", limelightManager.currentCameraArmAngle);
////        telemetry.addData("Cam Pose: ", limelightManager.camPose.getX(DistanceUnit.INCH) + ", " + limelightManager.camPose.getY(DistanceUnit.INCH) + ", " + limelightManager.camPose.getHeading(AngleUnit.DEGREES));
////        telemetry.addData("Robot Pose: ", limelightManager.robotPose.getX(DistanceUnit.INCH) + ", " + limelightManager.robotPose.getY(DistanceUnit.INCH) + ", " + limelightManager.robotPose.getHeading(AngleUnit.DEGREES));
////        telemetry.addData("Is valid?", limelightManager.validResult);
//        telemetry.addData("Green pos: ", limelightManager.greenMotifPos);
//
//        telemetry.update();
//    }
//
//
//
//
//
//
//
//    /**
//     * This method is called once at the init of the OpMode.
//     **/
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//
//        drive = new MecanumDrive(hardwareMap);
//        stateMachine = new ManagerStateMachine(hardwareMap, drive);
//        rotatorManager = stateMachine.getRotatorManager();
//        flywheelManager = stateMachine.getFlywheelManager();
//        colorSensorManager = stateMachine.getColorSensorManager();
//        intakeManager = stateMachine.getIntakeManager();
//        resetManager = stateMachine.getResetManager();
//        limelightManager = stateMachine.getLimelightManager();
//    }
//
//
//
//
//
//
//
//    /**
//     * This method is called continuously after Init while waiting for "play".
//     **/
//    @Override
//    public void init_loop() {
//        if (gamepad2.xWasPressed()) {
//            isRedAlliance = false;
//            buildPaths();
//            follower.setStartingPose(startPose);
//        } else if (gamepad2.bWasPressed()) {
//            isRedAlliance = true;
//            buildPaths();
//            follower.setStartingPose(startPose);
//        }
//
//        if (gamepad2.dpadDownWasPressed()) {
//            isWallSide = true;
//            buildPaths();
//            follower.setStartingPose(startPose);
//        } else if (gamepad2.dpadUpWasPressed()) {
//            isWallSide = false;
//            buildPaths();
//            follower.setStartingPose(startPose);
//        }
//
//
//        telemetry.addData("Auto: ", (isRedAlliance ? "Red " : "Blue ") + (isWallSide ? "wall" : "goal"));
//        telemetry.update();
//
//        setPathStateB(-1);
//        setPathStateC(-1);
//    }
//
//
//
//
//    /**
//     * This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system
//     **/
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathStateA(0);
//        setPathStateB(-1);
//
//        rotatorManager.startScript();
//        flywheelManager.startScript();
//        colorSensorManager.startScript();
//        resetManager.startScript();
//        limelightManager.startScript();
//
//        rotatorManager.rotatorPickupState = ManagerRotator.RotatorState.NEUTRAL;
//
//        limelightManager.moveCameraArm(90);
//    }
//
//
//
//
//    @Override
//    public void stop() {
//        rotatorManager.inRotator.setVelocity(0);
//        flywheelManager.pivotArm.setVelocity(0);
//        flywheelManager.flywheelLeft.setVelocity(0);
//        flywheelManager.flywheelRight.setVelocity(0);
//
//        intakeManager.intakeL.setPower(0);
//        intakeManager.intakeR.setPower(0);
//    }
//
//}