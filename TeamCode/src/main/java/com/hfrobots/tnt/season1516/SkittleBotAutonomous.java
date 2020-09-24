/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.season1516;


abstract class SkittleBotAutonomous extends SkittleBotTelemetry
{
    private RobotState currentRobotState;

    private boolean useEncoders;

    public SkittleBotAutonomous(boolean blueAlliance, boolean useEncoders) {
        this.useEncoders = useEncoders;
        ColorMatch matchBlue = new ColorMatch().redMin(0).redMax(0).
                blueMin(10).blueMax(20).greenMin(2).greenMax(4).alphaMin(0).alphaMax(40);
        ColorMatch matchRed = new ColorMatch().blueMin(0).blueMax(2).greenMin(0).greenMax(40).
                redMin(17).redMax(26).alphaMin(8).alphaMax(11);

        double powerWhenDetectingTape = .3;
        double powerWhenSeeking = .3;

        final ColorMatch matchMiddleAndRescue;

        if (blueAlliance) {
            matchMiddleAndRescue = matchBlue;
        } else {
            matchMiddleAndRescue = matchRed;
            // X-axis direction is reversed when we are in the red alliance
            powerWhenDetectingTape = -powerWhenDetectingTape;
            powerWhenSeeking = -powerWhenSeeking;
        }

        RobotState doneState = new DoneState(); // what the robot does when done (i.e nothing)

        RobotState startState = new StartState();
        RobotState driveToMiddleLine = new DriveAlongXAxisUntilColor("Drive to middle", powerWhenDetectingTape * 0.57, matchMiddleAndRescue);
        startState.setNextState(driveToMiddleLine);
        // Remember, y-axis driving is *always* positive with our program
        RobotState driveOffMiddleLine = new DriveAlongYAxisTimed("Drive off middle", Math.abs(powerWhenDetectingTape), 2500);
        driveToMiddleLine.setNextState(driveOffMiddleLine);
        RobotState driveUntilRescueRepairZone = new DriveAlongYAxisUntilColor("Drive to ResQ", Math.abs(powerWhenDetectingTape), matchMiddleAndRescue);
        driveOffMiddleLine.setNextState(driveUntilRescueRepairZone);
        RobotState driveIntoRescueRepairABit = new DriveAlongYAxisTimed("Drive into ResQ", .1, 500);
        driveUntilRescueRepairZone.setNextState(driveIntoRescueRepairABit);
        RobotState seekWhiteLine = new SeekWhiteLine("Find White Line", powerWhenSeeking);
        driveIntoRescueRepairABit.setNextState(seekWhiteLine);
        RobotState alignToBeacon = new DriveAlongXAxisTimed("Aligning to beacon", -0.1, 850);
        seekWhiteLine.setNextState(alignToBeacon);
        RobotState driveUntilWallTouch = new DriveAlongYAxisUntilTouchSensorPushed("To wall", 0.2);
        alignToBeacon.setNextState(driveUntilWallTouch);
        RobotState dumpClimbers = new DumpClimbers("Dump Climbers");
        driveUntilWallTouch.setNextState(dumpClimbers);
        dumpClimbers.setNextState(doneState);

        // Robot starts the state machine at the start state
        currentRobotState = startState;
    }

    @Override
    public void init() {
        super.init();

        if (!useEncoders) {
            runWithoutDriveEncoders();
        } else {
            runUsingEncoders();
        }

        // Set servos to initial required state
        setClimberDumpServoPosition(.5); // sets servo to stop position

        enableColorSensorLed(false); // bug with FTC software, must init() with LED off, on in start
    }

    /**
     * Implements a state machine that controls the robot during
     * autonomous operation.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override
    public void loop() {
        RobotState previousState = currentRobotState;
        currentRobotState = currentRobotState.doStuffAndGetNextState();
        // send the previous and current state names/date to the driver's station
        setFirstMessage("prev: " + previousState + " cur: " + currentRobotState);
        updateTelemetry();
    }

    /**
     * All robot "states" have this code to run
     */
    abstract class RobotState {
        protected RobotState nextState;
        protected String stateName;

        RobotState(String stateName) {
            this.stateName = stateName;
        }

        void setNextState(RobotState nextState) {
            this.nextState = nextState;
        }

        /**
         * Define what this state does, and what the next
         * state when the state is finished...
         */
        abstract RobotState doStuffAndGetNextState();

        @Override
        public String toString() {
            return stateName;
        }
    }

    class DoneState extends RobotState {
        DoneState() {
            super("Done");
        }

        RobotState doStuffAndGetNextState() {
            return this;
        }
    }

    class StartState extends RobotState {
        StartState() {
            super("Start");
        }

        RobotState doStuffAndGetNextState() {
            enableColorSensorLed(true);

            return nextState;
        }
    }

    class ColorMatch {
        private int redMin;
        private int redMax = Integer.MAX_VALUE;
        private int greenMin;
        private int greenMax = Integer.MAX_VALUE;
        private int blueMin;
        private int blueMax = Integer.MAX_VALUE;
        private int alphaMin;
        private int alphaMax = Integer.MAX_VALUE;

        ColorMatch redMin(int redMin) {
            this.redMin = redMin;

            return this;
        }

        ColorMatch redMax(int redMax) {
            this.redMax = redMax;

            return this;
        }

        ColorMatch greenMin(int greenMin) {
            this.greenMin = greenMin;

            return this;
        }

        ColorMatch greenMax(int greenMax) {
            this.greenMax = greenMax;

            return this;
        }

        ColorMatch blueMin(int blueMin) {
            this.blueMin = blueMin;

            return this;
        }

        ColorMatch blueMax(int blueMax) {
            this.blueMax = blueMax;

            return this;
        }

        ColorMatch alphaMin(int alphaMin) {
            this.alphaMin = alphaMin;

            return this;
        }

        ColorMatch alphaMax(int alphaMax) {
            this.alphaMax = alphaMax;

            return this;
        }

        boolean colorMatches(ColorSensorValues colorReading) {
            boolean match = false;

            if ((colorReading.red >= redMin && colorReading.red <= redMax)
                    && (colorReading.green >= greenMin && colorReading.green <= greenMax)
                    && (colorReading.blue >= blueMin && colorReading.blue <= blueMax)
                    && (colorReading.alpha >= alphaMin && colorReading.alpha <= alphaMax)) {
                match = true;
            }

            return match;
        }
    }

    abstract class ColorSensingState extends RobotState {
        private final ColorMatch colorMatch;
        private ColorSensorValues lastColorReading;

        ColorSensingState(String stateName, ColorMatch colorMatch) {
            super(stateName);
            this.colorMatch = colorMatch;
        }

        protected boolean colorMatches() {
            ColorSensorValues colorReading = getColorSensorValues();
            lastColorReading = colorReading;

            return colorMatch.colorMatches(colorReading);
        }

        @Override
        public String toString() {
            if (lastColorReading != null) {
                return stateName + " cr: " + lastColorReading;
            }

            return stateName;
        }
    }

    class DriveAlongXAxisUntilColor extends ColorSensingState {
        private final double motorPower;

        DriveAlongXAxisUntilColor(String stateName, double motorPower, ColorMatch colorMatch) {
            super(stateName, colorMatch);
            this.motorPower = motorPower;
        }

        RobotState doStuffAndGetNextState() {
            if (!colorMatches()) {
                driveAlongXAxis(motorPower);

                return this; // keep doing what we were doing
            } else {
                stopAllDriveMotors();
                return nextState;
            }
        }
    }

    class DriveAlongYAxisUntilColor extends ColorSensingState {
        private final double motorPower;

        DriveAlongYAxisUntilColor(String stateName, double motorPower, ColorMatch colorMatch) {
            super(stateName, colorMatch);
            this.motorPower = motorPower;
        }

        RobotState doStuffAndGetNextState() {
            if (!colorMatches()) {
                driveAlongYAxis(motorPower);

                return this; // keep doing what we were doing
            } else {
                stopAllDriveMotors();
                return nextState;
            }
        }
    }

    abstract class TimedDrive extends RobotState {
        private final long stopAfterMs;
        private long beginTimeMs = 0;
        private long lastElapsedTime = 0;

        public TimedDrive(String stateName, long stopAfterMs) {
            super(stateName);
            this.stopAfterMs = stopAfterMs;
        }

        RobotState doStuffAndGetNextState() {
            if (beginTimeMs == 0) {
                beginTimeMs = System.currentTimeMillis();
            } else {
                long now = System.currentTimeMillis();
                long elapsedTime = now - beginTimeMs;
                lastElapsedTime = elapsedTime;

                if (elapsedTime >= stopAfterMs) {
                    stopAllDriveMotors();

                    return nextState;
                }
            }

            doTheDriving();

            return this;
        }

        abstract void doTheDriving();

        @Override
        public String toString() {
            return stateName + " for: " + stopAfterMs + ", elapsed: " + lastElapsedTime;
        }
    }

    class DriveAlongYAxisTimed extends TimedDrive {
        private double motorPower;

        public DriveAlongYAxisTimed(String stateName, double motorPower, long stopAfterMs) {
            super(stateName, stopAfterMs);
            this.motorPower = motorPower;
        }

        @Override
        void doTheDriving() {
            driveAlongYAxis(motorPower);
        }
    }

    class DriveAlongXAxisTimed extends TimedDrive {
        private double motorPower;

        public DriveAlongXAxisTimed(String stateName, double motorPower, long stopAfterMs) {
            super(stateName, stopAfterMs);
            this.motorPower = motorPower;
        }

        @Override
        void doTheDriving() {
            driveAlongXAxis(motorPower);
        }
    }

    class SeekWhiteLine extends RobotState {

        long beginTime = 0;
        long reverseDirectionTimeMs = 3200;
        int seekDirectionMultiplier = 1;
        double powerWhenSeeking;
        boolean firstSeek = true;

        ColorMatch whiteLine = new ColorMatch().blueMin(25).blueMax(40).greenMin(25).greenMax(40).redMin(25).redMax(40).alphaMin(30).alphaMax(60);

        /**
         * Seeks back and forth along the X axis searching for the white line
         * in the ResQ repair zone.
         *
         * @param stateName Name of this state
         * @param powerWhenSeeking Initial power setting to use when seeking (including direction)
         */
        public SeekWhiteLine(String stateName, double powerWhenSeeking) {
            super(stateName);
            this.powerWhenSeeking = powerWhenSeeking;
        }

        @Override
        RobotState doStuffAndGetNextState() {
            if (beginTime == 0) {
                beginTime = System.currentTimeMillis();
            } else {
                long now = System.currentTimeMillis();
                long elapsedTime = now - beginTime;

                // Seek only a portion as long first cycle, because it's only 1/2 a cycle
                if (elapsedTime >= (/* firstSeek ? reverseDirectionTimeMs / 1.7 : */ reverseDirectionTimeMs)) {
                    firstSeek = false;
                    seekDirectionMultiplier = -seekDirectionMultiplier;
                    beginTime = System.currentTimeMillis();
                    reverseDirectionTimeMs = reverseDirectionTimeMs + (reverseDirectionTimeMs / 4);
                }
            }

            ColorSensorValues colorReading = getColorSensorValues();

            if (whiteLine.colorMatches(colorReading)) {
                stopAllDriveMotors();
                return nextState;
            }

            double drivePower = powerWhenSeeking * seekDirectionMultiplier;
            driveAlongXAxis(drivePower);

            return this;
        }
    }

    class DriveAlongYAxisUntilTouchSensorPushed extends RobotState {
        double motorPower;

        public DriveAlongYAxisUntilTouchSensorPushed(String stateName, double motorPower) {
            super(stateName);
            this.motorPower = motorPower;
        }

        @Override
        RobotState doStuffAndGetNextState() {
            if (isFrontTouchSensorPressed()) {
                stopAllDriveMotors();
                return nextState;
            }

            driveAlongYAxis(motorPower);

            return this;
        }
    }

    class DumpClimbers extends RobotState {
        private final int ARM_STATE_START = 0;
        private final int ARM_STATE_RAISING = 1;
        private final int ARM_STATE_HOLDING = 2;
        private final int ARM_STATE_LOWERING = 3;

        private int currentArmState = ARM_STATE_START;

        private long raiseBeginTimeMs;
        private long holdBeginTimeMs;
        private long lowerBeginTimeMs;
        private long raiseLowerElapsedTimeMs = 1000;

        public DumpClimbers(String stateName) {
            super(stateName);
        }

        @Override
        RobotState doStuffAndGetNextState() {
            switch (currentArmState) {
                case ARM_STATE_START:
                    raiseBeginTimeMs = System.currentTimeMillis();
                    setClimberDumpServoPosition(0); // up

                    currentArmState = ARM_STATE_RAISING;

                    return this;
                case ARM_STATE_RAISING:

                    long now = System.currentTimeMillis();
                    long elapsedTime = now - raiseBeginTimeMs;

                    if (elapsedTime >= raiseLowerElapsedTimeMs) {
                        setClimberDumpServoPosition(.5);
                        holdBeginTimeMs = System.currentTimeMillis();

                        currentArmState = ARM_STATE_HOLDING;

                        return this;
                    }

                    currentArmState = ARM_STATE_RAISING;

                    return this;
                case ARM_STATE_HOLDING:
                    now = System.currentTimeMillis();
                    elapsedTime = now - holdBeginTimeMs;

                    if (elapsedTime >= 500) {
                        setClimberDumpServoPosition(1); // down
                        lowerBeginTimeMs = System.currentTimeMillis();
                        currentArmState = ARM_STATE_LOWERING;

                        return this;
                    }

                    currentArmState = ARM_STATE_HOLDING;

                    return this;
                case ARM_STATE_LOWERING:
                    now = System.currentTimeMillis();
                    elapsedTime = now - lowerBeginTimeMs;

                    if (elapsedTime >= raiseLowerElapsedTimeMs) {
                        setClimberDumpServoPosition(0.5); // stop
                        return nextState;
                    }

                    currentArmState = ARM_STATE_LOWERING;

                    return this;
                default:
                    // error, try next state?
                    return nextState;
            }
        }
    }
}

