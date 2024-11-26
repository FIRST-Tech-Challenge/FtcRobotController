package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class ATeleop extends LinearOpMode {

    RobotMain bart;


    /**CAMERA**/
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private boolean cameraIsOn;

    final double xP = 0.05;
    final double yP = 0.05;
    final double thetaP = 0.05;
    final double angularP = 0;

    double inputtedY = 12;

    enum State {
        MANUAL,
        GO_TO_INPUTTED_Y,
        LINE_UP_INTAKE_UNTIL_USER_CONFIRMS,
        INTAKE,
        DRIVE_TO_BUCKET,
        RELOCALIZE
    }

    State currentState;
    State previousState;

    ElapsedTime elapsedTime;

    double loopTime;
    double maxLoopTime;
    double minLoopTime;
    double avgLoopTime;
    double totalTime;
    double totalLoops;

    double targetAngle;
    double automatedAngularVel;

    GamepadEx playerOne, playerTwo;

    @Override
    public void runOpMode() throws InterruptedException {
        //Camera
        //initAprilTag();
        loopTime = 0;
        maxLoopTime = 0;
        minLoopTime = 99999;
        avgLoopTime = 0;
        totalTime = 0;
        totalLoops = 0;


        targetAngle = 0;
        automatedAngularVel = 0;

        //create robot
        bart = new RobotMain(hardwareMap, telemetry);

        //Create Gamepads
        playerOne = new GamepadEx(gamepad1);
        playerTwo = new GamepadEx(gamepad2);

        //Bulk Cache Read
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();
        bart.output.setComponentPositionsFromSavedPosition("rest");

        currentState = State.MANUAL;
        previousState = currentState;

        //visionPortal.stopStreaming();
        cameraIsOn = false;

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (opModeIsActive()) {

            //READ
            playerOne.readButtons();
            playerTwo.readButtons();
            bart.readHubs();
            bart.mecanaDruve.updatePoseEstimate();

            //UPDATE
            stateMachine();

            //WRITE
            bart.writeAllComponents();

            /** TELEMETRY **/

            telemetry.addLine(bart.output.currentPosition().pointTelemetry());
            telemetry.addData("\n", bart.output.currentPosition().componentValuesIrl());
            telemetry.addLine(bart.output.wrist.toString());
            telemetry.addData("wristServosLeft", bart.output.wrist.getCurrentLeftServoPosition());
            telemetry.addData("wristServosRight", bart.output.wrist.getCurrentRightServoPosition());
            telemetry.addData("armServos", bart.output.arm.getCurrentServoPosition());
            loopTime = elapsedTime.milliseconds() - totalTime;
            if (loopTime > maxLoopTime) maxLoopTime = loopTime;
            if (loopTime < minLoopTime) minLoopTime = loopTime;
            totalLoops++;
            totalTime = elapsedTime.milliseconds();
            avgLoopTime = totalTime/totalLoops;
            telemetry.addData("Loop Time", loopTime);
            telemetry.addData("Max Loop Time", maxLoopTime);
            telemetry.addData("Min Loop Time", minLoopTime);
            telemetry.addData("Avg Loop Time", avgLoopTime);
            telemetry.update();

        }



    }



    public void stateMachine() {
        switch (currentState) {
            case MANUAL:
                //LOGIC
                manualControl();


                //EVENTS
                if (playerOne.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    thogLockToNextCounterClockwise();
                }
                if (playerOne.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    thogLockToNextClockwise();
                }
                if (playerOne.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
                    currentState = State.GO_TO_INPUTTED_Y;
                }

                break;

            case GO_TO_INPUTTED_Y:
                //LOGIC
                pControllerToPos(new Pose2d(30, inputtedY, Math.toRadians(180)));

                //EVENTS
                if (atPos(new Pose2d(30, inputtedY, Math.toRadians(180)))) {
                    currentState = State.LINE_UP_INTAKE_UNTIL_USER_CONFIRMS;
                }
                checkIfKilled();
                break;

            case LINE_UP_INTAKE_UNTIL_USER_CONFIRMS:
                //LOGIC
                bart.driveRobotRelative(0, playerOne.getLeftX(), 0);

                //EVENTS
                if(playerOne.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    currentState = State.INTAKE;
                }
                checkIfKilled();
                break;

            case INTAKE:
                //LOGIC
                bart.driveFieldRelative(getXSpeed(20), playerOne.getLeftX(), 0);

                //once we get there do the intaking until we get one
                //do it later
                //for now just press b to move on
                if (playerOne.wasJustPressed(GamepadKeys.Button.B)) {
                    currentState = State.DRIVE_TO_BUCKET;
                }

                //when switching to DRIVE_TO_BUCKET switch on visionPortal and set cameraOn to true

                checkIfKilled();

                break;

            case DRIVE_TO_BUCKET:
                //LOGIC
                if (bart.mecanaDruve.pose.position.x < 30) {
                    bart.driveFieldRelative(0.5, 0, 0);
                } else {
                    //pControllerToPos(
                    //        new Pose2d(RobotMain.BUCKET_POINT.getPointBlue().toVector2d(),
                    //        RobotMain.BUCKET_POINT.getPointBlue().angleBetweenPoints(bart.robotPosAsPoint2d())));
                    //Localize with April Tags
                    //bart.mecanaDruve.setPosFromOutside(localize());
                }


                //rasie the amr and stuff to high bucket
                //be able to swtich to low bucket if we fill it all up
                //and we wanna

                //do the relocalizing here

                //EVENTS
                if (atPos(new Pose2d(RobotMain.BUCKET_POINT.getPointBlue().toVector2d(),
                        RobotMain.BUCKET_POINT.getPointBlue().angleBetweenPoints(bart.robotPosAsPoint2d())))) {
                    //open gripper
                    currentState = State.GO_TO_INPUTTED_Y;
                    //visionPortal.stopStreaming();
                    cameraIsOn = false;

                }
                checkIfKilled();
                break;

            case RELOCALIZE:
                //just doing that, nothgin else
                break;

                //do later alan thign if we do
        }
    }


    public void thogLockToNextClockwise() {
        if (Math.toDegrees(bart.mecanaDruve.pose.heading.toDouble()) < -90) {
            targetAngle = -90;
        } else if (Math.toDegrees(bart.mecanaDruve.pose.heading.toDouble()) < 0) {
            targetAngle = 0;
        } else if (Math.toDegrees(bart.mecanaDruve.pose.heading.toDouble()) < 90) {
            targetAngle = 90;
        } else if (Math.toRadians(bart.mecanaDruve.pose.heading.toDouble()) < 180) {
            targetAngle = 180;
        }
    }

    public void thogLockToNextCounterClockwise() {
        if (Math.toDegrees(bart.mecanaDruve.pose.heading.toDouble()) > 90) {
            targetAngle = 90;
        } else if (Math.toDegrees(bart.mecanaDruve.pose.heading.toDouble()) > 0) {
            targetAngle = 0;
        } else if (Math.toDegrees(bart.mecanaDruve.pose.heading.toDouble()) > -90) {
            targetAngle = -90;
        } else if (Math.toRadians(bart.mecanaDruve.pose.heading.toDouble()) > -180) {
            targetAngle = -180;
        }
    }

    public void goToTargetAngle() {
        automatedAngularVel = (angularP * targetAngle) / 180;
    }


    public void pControllerToPos(Pose2d desiredPos) {
        Pose2d difference = desiredPos.minusExp(bart.mecanaDruve.pose);
        //desiredPos.minusExp()
        //double xDiff = desiredPos.position.x - bart.mecanaDruve.pose.position.x;
        //double yDiff = desiredPos.position.y - bart.mecanaDruve.pose.position.y;
        //double thetaDiff = desiredPos.heading.toDouble() -


            bart.driveFieldRelative(
                    difference.position.x*xP,
                    difference.position.y*yP,
                    difference.heading.toDouble()*thetaP);

    }

    public double getXSpeed(double desiredX) {
        return  (desiredX-bart.mecanaDruve.pose.position.x)*xP;
    }

    public boolean atPos(Pose2d desiredPos) {
        return  RobotMath.isAbsDiffWithinRange(desiredPos.position.x, bart.mecanaDruve.pose.position.x, 0.5) &&
                RobotMath.isAbsDiffWithinRange(desiredPos.position.y, bart.mecanaDruve.pose.position.y, 0.5) &&
                RobotMath.isAbsDiffWithinRange(desiredPos.heading.toDouble(), bart.mecanaDruve.pose.heading.toDouble(), Math.toRadians(5));
    }


    public void manualControl() {
        //bart.driveRobotRelative(playerOne.getLeftY(), playerOne.getLeftX(), playerOne.getRightX());
        if (playerOne.getRightX() == 0) {
            bart.mecanaDruve.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y*0.7,
                            -gamepad1.left_stick_x*0.7
                    ),
                    targetAngle*0.7
            ));
        } else {
            targetAngle -= gamepad1.right_stick_x*0.7;
            bart.mecanaDruve.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y*0.7,
                            -gamepad1.left_stick_x*0.7
                    ),
                    automatedAngularVel*0.7
            ));
        }




        if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            bart.output.setComponentPositionsFromSavedPosition("aboveTransferOpen");
            bart.intake.closeGate();
        }
        if (playerTwo.isDown(GamepadKeys.Button.A)) {
            /*if (bart.intake.isGateOpen()) {
                bart.output.calculateComponentPositionsFromSavedEndPoint("transfer");
            }
            bart.intake.openGate();*/
            bart.transfer();
        }
        if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            bart.intake.openGate();
        }

        //OUTPUT CONTROL
        //manual output control
        if (playerTwo.getRightY() != 0) {
            bart.output.verticalSlides.setSlidePower(0.5 * playerTwo.getRightY());
            bart.output.setTargetToCurrentPosition();
        } else {
            //DONE AUTO
            if (playerTwo.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bart.output.setComponentPositionsFromSavedPosition("grab");
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.Y)) {
                bart.output.setComponentPositionsFromSavedPosition("highBucket");
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.B)) {
                //bart.output.setComponentPositionsFromSavedPosition("lowBucket");
                bart.output.setComponentPositionsFromSavedPosition("highBarFront");
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.X)) {
                bart.output.setComponentPositionsFromSavedPosition("highBarBack");
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bart.output.setComponentPositionsFromSavedPosition("level1AscentTeleop");
            }

            bart.output.sendVerticalSlidesToTarget();

        }

        if (playerTwo.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            bart.output.gripper.flipFlop();
        }

        /*if (playerTwo.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            bart.output.calculateComponentPositions(
                    new OutputEndPoint(new Point2d(7.7, 18.4), 0, 0, false)
            );
        }*/
        //reset gyro
        if (playerOne.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            //bart.resetIMU();
        }
        //switch drive mode (robot-oriented default, switch to field-oriented)
        if (playerOne.wasJustPressed(GamepadKeys.Button.Y)) {
            bart.switchDriveMode();
        }

        //toggle camera
        if (playerOne.wasJustPressed(GamepadKeys.Button.A)) {
            if (cameraIsOn) {
                //visionPortal.stopStreaming();
            } else {
                //visionPortal.resumeStreaming();
            }
        }

        //this is what we will almost always do, only don't when manually going down to reset encoder
        /*if (!playerTwo.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            bart.output.sendVerticalSlidesToTarget();
        } else {
            bart.output.verticalSlides.setSlidePower(0.5*playerTwo.getRightY());
        }
        if (playerTwo.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            bart.resetEncoders();
        }*/



        //intake
        if (playerTwo.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            bart.intake.setHorizontalSlideToSavedPosition("transfer");
        } else if (playerTwo.isDown(GamepadKeys.Button.DPAD_UP)) {
            bart.intake.setHorizontalSlideToSavedPosition("max");
        } else if (!playerTwo.isDown(GamepadKeys.Button.A)){
            bart.intake.setHorizontalSlidePower(playerTwo.getLeftY() * 1);
        }

        if (playerTwo.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) {
            bart.intake.setIntakeMotorPower(playerTwo.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        } else if (playerTwo.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
            bart.intake.setIntakeMotorPower(playerTwo.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * (-1));
        } else if (!playerTwo.isDown(GamepadKeys.Button.A)){
            bart.intake.setIntakeMotorPower(0);
        }

    }

    public void checkIfKilled() {
        if (!playerOne.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            previousState = currentState;
            currentState = State.MANUAL;
        }
    }
    public void doPlayerOnesChecksEachFrame() {
        checkIfKilled();
        inputtedY = RobotMain.dpadInputToChangeValueUpIsNegative(inputtedY, playerOne);
    }

    /**
     * Initialize the AprilTag processor.
     */
    /*private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
*/
/*
    private Pose2d localize() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                detection.robotPose.getOrientation();
                return new Pose2d(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw());
            }
        }
        return bart.mecanaDruve.pose;
    }*/
}


