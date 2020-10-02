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

package com.hfrobots.tnt.corelib.drive;

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * State machine state that will turn a TankDrive a relative number of degrees
 * (based on code from the FTC SDK Pushbot example)
 */
public class GyroTurnState extends TimeoutSafetyState {
    private static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable

    private final TankDrive drive;
    private final ModernRoboticsI2cGyro gyro;
    private final Turn turn;
    private final double initialPower;

    private int targetHeading = Integer.MIN_VALUE;

    public GyroTurnState(String name, TankDrive drive,
                            ModernRoboticsI2cGyro gyro,
                            Turn turn,
                            Telemetry telemetry,
                            double initialPower,
                            long safetyTimeoutMillis) {
        super(name, telemetry, safetyTimeoutMillis);
        this.drive = drive;
        this.gyro = gyro;
        this.turn = turn;
        this.initialPower = initialPower;
    }

    @Override
    public State doStuffAndGetNextState() {

        if (targetHeading == Integer.MIN_VALUE) {
            // not-yet initialized
            int currentHeading = gyro.getIntegratedZValue();
            int turnHeading = turn.getHeading();
            targetHeading = currentHeading + turnHeading;
            Log.d("VV", "Gyro turn: current heading: " + currentHeading + ", relative turn heading: " + turnHeading + ", target heading: " + targetHeading);
        }

        if (onHeading(initialPower, targetHeading)) {
            Log.d("VV", "Gyro turn heading reached - stopping drive");

            return nextState;
        } else if (isTimedOut()) {
            Log.e("VV", "Gyro turn timeout reached - stopping drive");

            drive.drivePower(0, 0);
        }

        return this;
    }

    @Override
    public void resetToStart() {
        super.resetToStart();
    }

    @Override
    public void liveConfigure(NinjaGamePad gamePad) {

    }

    // re-use of Pushbot gyro steer

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @return
     */
    boolean onHeading(double speed, double angle) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            // Note - this is true tank steering, one side moves directly opposite the other!
            steer = getSteer(error);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;

            if (Math.abs(rightSpeed) < 0.2) { // cutoff for our robot right now, it won't move
                leftSpeed = 0.0;
                rightSpeed = 0.0;
                onTarget = true;
            }
        }

        // Send desired speeds to motors.
        drive.drivePower(leftSpeed, rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @return
     */
    public double getSteer(double error) {
        return Range.clip(error * P_TURN_COEFF, -1, 1);
    }
}
