package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020.EbotsRev2mDistanceSensor;
import org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020.EbotsRevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;
import org.firstinspires.ftc.teamcode.ultimategoal2020.EncoderTracker;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.gameelements.PlayField;
import org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020.EbotsColorSensor;
import org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020.EbotsDigitalTouch;

import java.util.ArrayList;

public class StatePrematchSetup extends AbstractAutonState {
    /**
     * This method defines a State intended to be run as a state machine
     * It implements AutonState interface
     *
     * Note that if right bumper is pressed, Telemetry screen is toggled for detailed
     * information about the encoder readings, which can be used for manual calibration
     */


    boolean isSetupCorrect;
    boolean wasSetupCorrect = false;
    boolean isSetupStable = false;
    StopWatch setupStopWatch = new StopWatch();
    long previousLoopEnd;
    RobotSide robotSide;
    EbotsColorSensor.TapeColor tapeColor;
    EbotsRevBlinkinLedDriver ledDriver;
    RevBlinkinLedDriver.BlinkinPattern patternPositionVerified = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    double nominalDistance;
    double distanceTolerance = 10;
    double actualDistance;

    TelemetryScreen telemetryScreen = TelemetryScreen.A;
    StopWatch lockoutTimer = new StopWatch();
    long buttonLockoutLimit = 750L;

    final boolean debugOn = true;
    final String logTag = "EBOTS";

    private enum TelemetryScreen{
        A,B
    }

    // ***********   CONSTRUCTOR   ***********************
    public StatePrematchSetup(LinearOpMode opModeIn, EbotsRobot2020 robotIn, Class<? extends AbstractAutonState> nextAutonState){
        // Call the generic constructor from the super class (AbstractAutonState) to initialize opmode, robot, nextAutonStateClass
        super(opModeIn, robotIn, nextAutonState);

        ledDriver = EbotsRevBlinkinLedDriver.getEbotsRevBlinkinLedDriverByLedLocation(EbotsRevBlinkinLedDriver.LedLocation.MAIN, robot.getLedDrivers());

    }

    // ***********   GETTERS    ***********************

    // NOTE: there are default getters in AbstractAutonState for
    //      getCurrentAutonState
    //      getNextAutonState


    // ***********   INTERFACE METHODS   ***********************
    @Override
    public boolean areExitConditionsMet() {
        // Exit if either:
        //  a) setup is stable OR
        //  b) opMode is started
        //  c) manual override
        long stableSetupTimeTarget = 5000L;
        boolean isCorrectSetupStable = setupStopWatch.getElapsedTimeMillis() > stableSetupTimeTarget;

        // Verify that the setup is stable:
        //  a) setup is stable
        //      i) that it is correct AND
        //      ii) has been correct for some period of time (stableSetupTimeTarget)
        isSetupStable = (isSetupCorrect & isCorrectSetupStable);

        //  c) manual override
        //      i) Left Bumper and X
        //      ii) Opmode has been started for at least 1.5s (so don't get input from previous opmode exit gamepad input)
        boolean manualOverride = (opMode.gamepad1.left_bumper && opMode.gamepad1.x && setupStopWatch.getElapsedTimeMillis()>1500) ? true : false;

        boolean verdict = (isSetupStable || this.opMode.isStarted() || manualOverride);
        return verdict;
    }

    @Override
    public void performStateSpecificTransitionActions() {
        //If exiting because setup is stable, Perform a light show to verify exiting
        if(isSetupStable) {
            StopWatch blinkTimer = new StopWatch();
            long blinkTimeLimit = 350L;
            int numBlinks = 3;

            for (int i = 0; i < numBlinks; i++) {
                ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                while (!opMode.isStarted() && !opMode.isStopRequested() && blinkTimer.getElapsedTimeMillis() < blinkTimeLimit) {
                    //just wait
                }
                blinkTimer.reset();
                ledDriver.setPattern(patternPositionVerified);
                while (!opMode.isStarted() && !opMode.isStopRequested() && blinkTimer.getElapsedTimeMillis() < blinkTimeLimit) {
                    //just wait
                }
                blinkTimer.reset();
            }
        }

        // Set Alliance color
        ledDriver.setAlliancePattern(robot.getAlliance());

        // Zero the encoders
        robot.zeroEncoders();
    }

    @Override
    public void performStateActions() {
        //if(debugOn) Log.d(logTag, currentAutonStateEnum + ": entering performStateActions");
        // Update the readings from the sensors
        performSensorHardwareReads();
        // Determine robotSide, tapeColor, and alliancePattern
        assignMeasurementParameters();

        // Now check if three conditions are met
        //  1) Touching the back wall
        //  2) Positioned over the startline tape
        //  3) On the correct startline
        boolean isOnWall = isTouchingBackWall();
        boolean isOnTape = isCorrectRobotSideOnCorrectColorTape();
        boolean isCorrectStartLine = isRobotPlacedOnCorrectStartLine();

        // Provide the verdict for setup correctness for this pass through loop
        isSetupCorrect = isOnTape && isOnWall && isCorrectStartLine;

        // If the setup just turned from incorrect to correct, reset the setupStopWatch
        // Exit conditions require the setup to be stable for some period of time
        // which is measured with the setupStopWawtch
        if (isSetupCorrect & !wasSetupCorrect){
            setupStopWatch.reset();
        }

        // Use this to pass to the next loop iteration
        wasSetupCorrect = isSetupCorrect;

        // switch telemetry screens when press bumpers
        if((lockoutTimer.getElapsedTimeMillis() > buttonLockoutLimit)
                && (opMode.gamepad1.left_bumper | opMode.gamepad1.right_bumper)){
            shiftTelemetryScreen();
            lockoutTimer.reset();
        }

        updateTelemetry();

        updateLedDisplay();

        previousLoopEnd = setupStopWatch.getElapsedTimeMillis();

    }

    // ***********   CLASS MEMBER METHODS   ***********************
    private void updateLedDisplay() {
        //display LED lights, green is good to go, red means there is a problem in setup
        if(isSetupCorrect){
            ledDriver.setPattern(patternPositionVerified);
        } else{
            ledDriver.setAlliancePattern(robot.getAlliance());
        }
    }

    private boolean isTouchingBackWall() {
        EbotsDigitalTouch backWall = EbotsDigitalTouch.getEbotsDigitalTouchByButtonFunction(EbotsDigitalTouch.ButtonFunction.DETECT_BACK_WALL,
                robot.getEbotsDigitalTouches());
        return backWall.getIsPressed();
    }

    private boolean isCorrectRobotSideOnCorrectColorTape() {
        return EbotsColorSensor.isSideOnColor(robot.getEbotsColorSensors(), robotSide, tapeColor);
    }

    private boolean isRobotPlacedOnCorrectStartLine() {
        boolean isCorrectStartLine = false;

        //calculate expected distance from the wall the distance sensor to the wall
        double playFieldHalfWidth = new PlayField().getFieldXSize() /2;

        // this is refactored to use the robot's currentPose, which was set based on the startLine
        // double nominalDistance = playFieldHalfWidth - Math.abs(startLineY) - robotWidth;
        nominalDistance = playFieldHalfWidth - Math.abs(robot.getActualPose().getY()) -
                (robot.getSizeCoordinate(CsysDirection.Y) / 2);

        // set which distance sensor will provide the reading
        RobotSide distanceSide = (robot.getAlliance() == Alliance.RED) ? RobotSide.RIGHT : RobotSide.LEFT;
        actualDistance = EbotsRev2mDistanceSensor.getDistanceForRobotSide(distanceSide, robot.getEbotsRev2mDistanceSensors());

        //check distance for robot side
        if (actualDistance >= (nominalDistance - distanceTolerance)
                && actualDistance <= (nominalDistance + distanceTolerance)){
            isCorrectStartLine = true;
        }

        return isCorrectStartLine;
    }


    private void assignMeasurementParameters() {
        // Check alliance
        if (robot.getAlliance() == Alliance.RED) {
            robotSide = RobotSide.LEFT;
            tapeColor = EbotsColorSensor.TapeColor.RED;
        } else {
            robotSide = RobotSide.RIGHT;
            tapeColor = EbotsColorSensor.TapeColor.BLUE;
        }
    }


    private void performSensorHardwareReads() {
        long loopDuration = setupStopWatch.getElapsedTimeMillis() - previousLoopEnd;
        robot.bulkReadSensorInputs(loopDuration,true,true);
        robot.updateAllSensorValues();

//        //Read the values for the color sensors from hardware into variables
//        for (EbotsColorSensor sensor : robot.getEbotsColorSensors()) {
//            sensor.setColorValue();
//        }
//
//        //Read the values for the distance sensors from hardware into variables
//        for(EbotsRev2mDistanceSensor distanceSensor: robot.getEbotsRev2mDistanceSensors()){
//            distanceSensor.setDistanceInches();
//        }
//
//        //Read the values for the digital touch sensors
//        for(EbotsDigitalTouch ebotsDigitalTouch: robot.getEbotsDigitalTouches()){
//            ebotsDigitalTouch.setIsPressed();
//        }
    }

    private void shiftTelemetryScreen(){
        if(telemetryScreen == TelemetryScreen.A){
            telemetryScreen = TelemetryScreen.B;
        } else{
            telemetryScreen = TelemetryScreen.A;
        }
        opMode.telemetry.clearAll();
    }


    private void updateTelemetry(){
        if(telemetryScreen == TelemetryScreen.A) {
            this.opMode.telemetry.addLine("Current autonState: " + this.getClass().getSimpleName());
            this.opMode.telemetry.addLine("opMode is Started / Active: " + opMode.isStarted() + "/" + opMode.opModeIsActive());
            this.opMode.telemetry.addLine("Setup Config: " + robot.getAlliance() + " | " + robot.getActualPose().toString());
            this.opMode.telemetry.addLine("Actual -- Expected[<-->]: " +
                    String.format("%.2f", (this.actualDistance)) + " -- " +
                    String.format("%.2f", (nominalDistance - distanceTolerance)) + "<-->" +
                    ", " + String.format("%.2f", (nominalDistance + distanceTolerance)));
            this.opMode.telemetry.addLine("Robot is on the back wall: " + isTouchingBackWall());
            this.opMode.telemetry.addLine("Robot is on the correct tape: " + isCorrectRobotSideOnCorrectColorTape());
            this.opMode.telemetry.addLine("Robot is on the correct start line: " + isRobotPlacedOnCorrectStartLine());
            this.opMode.telemetry.addLine("Overall correct set up: " + isSetupCorrect + " - " + setupStopWatch.toString());
            ArrayList<EbotsRevBlinkinLedDriver> ledDrivers = robot.getLedDrivers();
            EbotsRevBlinkinLedDriver ledDriver = EbotsRevBlinkinLedDriver.getEbotsRevBlinkinLedDriverByLedLocation(EbotsRevBlinkinLedDriver.LedLocation.MAIN, ledDrivers);
            this.opMode.telemetry.addLine("LED pattern: " + ledDriver.getLedLocation());
        } else {
            // Read out the encoders and report the values in telemetry
            opMode.telemetry.addData("Heading", robot.getActualPose().getHeadingDeg());
            for(EncoderTracker e: robot.getEncoderTrackers()){
                opMode.telemetry.addLine(e.toString());
            }
        }
        this.opMode.telemetry.update();

    }
}
