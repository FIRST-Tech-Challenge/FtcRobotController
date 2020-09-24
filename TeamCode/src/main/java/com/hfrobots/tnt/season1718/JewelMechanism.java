/**
 Copyright (c) 2017 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season1718;

import android.util.Log;

import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.state.DelayState;
import com.hfrobots.tnt.corelib.state.State;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.concurrent.TimeUnit;

/**
 * All of the code that knows how to operate the jewel mechanism
 */
public class JewelMechanism {

    private final static double SENSOR_STOWED_POSITION = 0;

    private final static double SENSOR_DEPLOYED_POSITION = 0.345;

    private final Servo sensorServo;

    private final LynxI2cColorRangeSensor jewelSensor;

    private final Rotation turnDirectionTowardsColorSensor;

    public JewelMechanism(Servo sensorServo, LynxI2cColorRangeSensor jewelSensor,
                          Rotation turnDirectionTowardsColorSensor) {
        this.sensorServo = sensorServo;
        this.jewelSensor = jewelSensor;
        this.turnDirectionTowardsColorSensor = turnDirectionTowardsColorSensor;
        stowSensor();
    }

    public void deploySensor() {
        sensorServo.setPosition(SENSOR_DEPLOYED_POSITION);
    }

    public void stowSensor() {
       sensorServo.setPosition(SENSOR_STOWED_POSITION);
    }

    public JewelMechanismStowSensorState getStowSensorState(Telemetry telemetry) {
        return new JewelMechanismStowSensorState("stowing sensor", telemetry);
    }

    public JewelMechanismDeploySensorState getDeploySensorState(Telemetry telemetry) {
        return new JewelMechanismDeploySensorState("deploying sensor", telemetry);
    }

    public JewelMechanismDetectAndTurn getDetectAndTurnState(Telemetry telemetry, Constants.Alliance alliance, MecanumDrive drive) {
        return new JewelMechanismDetectAndTurn("detect and turn", telemetry, alliance,drive);
    }

    public JewelMechanismDetectAndTurnWithMoreStuff getDetectAndTurnStateWithMoreStuff(Telemetry telemetry, Constants.Alliance alliance, LynxEmbeddedIMU imu, MecanumDrive drive) {
        return new JewelMechanismDetectAndTurnWithMoreStuff("detect and turn", telemetry, alliance, imu, drive);
    }

    public Rotation getTurnDirectionForDetectedJewel(Constants.Alliance alliance) {
        int blue = jewelSensor.blue();
        int red = jewelSensor.red();
        int green = jewelSensor.green();

        Log.d(Constants.LOG_TAG, "jewel mech detected RGB " + red + " " + green + " " + blue + " for alliance " + alliance);


        if (Math.abs(blue-red) < 4) {
            return null;
        }

        if (blue > red) {

            Log.d(Constants.LOG_TAG, "jewel mech thinks we saw blue");
            // we saw blue
            if (alliance.equals(Constants.Alliance.BLUE)) {
                Rotation turn = Turn.invert(turnDirectionTowardsColorSensor);
                Log.d(Constants.LOG_TAG, "jewel mech turning " + turn);

                return turn;
            } else {
                Log.d(Constants.LOG_TAG, "jewel mech turning " + turnDirectionTowardsColorSensor);
                return turnDirectionTowardsColorSensor; // towards
            }
        } else if (red > blue) {
            // we saw red
            Log.d(Constants.LOG_TAG, "jewel mech thinks we saw red");

            if (alliance.equals(Constants.Alliance.RED)) {
                Rotation turn = Turn.invert(turnDirectionTowardsColorSensor);
                Log.d(Constants.LOG_TAG, "jewel mech turning " + turn);

                return turn;
            } else {
                Log.d(Constants.LOG_TAG, "jewel mech turning " + turnDirectionTowardsColorSensor);
                return turnDirectionTowardsColorSensor;
            }
        } else {
            return null;
        }
    }

    class JewelMechanismDeploySensorState extends State {
        JewelMechanismDeploySensorState(String name, Telemetry telemetry) {
            super(name, telemetry);
        }
        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }

        public State doStuffAndGetNextState() {
            deploySensor();
            return nextState;
        }
    }

    class JewelMechanismStowSensorState extends State {
        JewelMechanismStowSensorState(String name, Telemetry telemetry) {
            super(name, telemetry);
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }

        public State doStuffAndGetNextState() {
            stowSensor();
            return nextState;
        }
    }

    class JewelMechanismDetectAndTurn extends State {
        Constants.Alliance alliance;
        boolean detected = false;
        MecanumTimedTurnState turnState = null;
        final MecanumDrive drive;

        JewelMechanismDetectAndTurn(String name, Telemetry telemetry, Constants.Alliance alliance, MecanumDrive drive) {
            super(name, telemetry);
            this.alliance = alliance;
            this.drive = drive;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }

        public State doStuffAndGetNextState() {
            if (!detected) {
                Rotation directionToTurn = getTurnDirectionForDetectedJewel(this.alliance);
                detected = true;

                if (directionToTurn == null) {
                    JewelMechanism.JewelMechanismStowSensorState stowSensorState = getStowSensorState(telemetry);
                    State waitToStow = new DelayState( "waiting for stow", telemetry,  2);
                    stowSensorState.setNextState(waitToStow);
                    waitToStow.setNextState(nextState);

                    return stowSensorState;
                }

                // TODO: This needs to handle turning, stowing, waiting, "un"-turning
                turnState = new MecanumTimedTurnState("turn for jewel", telemetry, drive, directionToTurn, 250);
                turnState.setNextState(nextState);
            }

            return turnState.doStuffAndGetNextState();
        }
    }

    class JewelMechanismDetectAndTurnWithMoreStuff extends State {
        Constants.Alliance alliance;
        boolean detected = false;
        State turnState = null;
        final MecanumDrive drive;
        final LynxEmbeddedIMU imu;

        JewelMechanismDetectAndTurnWithMoreStuff(String name, Telemetry telemetry, Constants.Alliance alliance, LynxEmbeddedIMU imu, MecanumDrive drive) {
            super(name, telemetry);
            this.alliance = alliance;
            this.drive = drive;
            this.imu = imu;
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }

        public State doStuffAndGetNextState() {
            if (!detected) {
                Rotation directionToTurn = getTurnDirectionForDetectedJewel(this.alliance);
                detected = true;

                if (directionToTurn == null) {
                    JewelMechanism.JewelMechanismStowSensorState stowSensorState = getStowSensorState(telemetry);
                    State waitToStow = new DelayState( "waiting for stow", telemetry,  2);
                    stowSensorState.setNextState(waitToStow);
                    waitToStow.setNextState(nextState);

                    return stowSensorState;
                }

                Turn turn = new Turn(directionToTurn, 10);

                MecanumGyroTurnState.Builder turnBuilder = MecanumGyroTurnState.builder();
                turnBuilder.setTurn(turn).setImu(imu).setMecanumDrive(drive).setPLargeTurnCoeff(RobotConstants.P_LARGE_TURN_COEFF)
                        .setPSmallTurnCoeff(RobotConstants.P_SMALL_TURN_COEFF).setName("Turn towards opposing jewel")
                        .setSafetyTimeoutMillis(TimeUnit.SECONDS.toMillis(15)).setMaxPower(0.2D);

                turnState = turnBuilder.build();

                JewelMechanism.JewelMechanismStowSensorState stowSensorState = getStowSensorState(telemetry);
                turnState.setNextState(stowSensorState);
                State waitToStow = new DelayState( "waiting for stow", telemetry,  2);
                stowSensorState.setNextState(waitToStow);

                turnBuilder = MecanumGyroTurnState.builder();
                turnBuilder.setTurn(turn.invert()).setImu(imu).setMecanumDrive(drive).setPLargeTurnCoeff(RobotConstants.P_LARGE_TURN_COEFF)
                        .setPSmallTurnCoeff(RobotConstants.P_SMALL_TURN_COEFF).setName("Turn away from opposing jewel")
                        .setSafetyTimeoutMillis(TimeUnit.SECONDS.toMillis(15)).setMaxPower(0.2D);;

                State turnBackState = turnBuilder.build();
                waitToStow.setNextState(turnBackState);
                turnBackState.setNextState(nextState);
            }

            return turnState.doStuffAndGetNextState();
        }
    }

}
