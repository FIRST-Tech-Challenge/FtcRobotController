/**
 Copyright (c) 2018 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season1819;

import android.util.Log;

//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A TNT State Machine state that will follow a RoadRunner trajectory
 * using motion profiles for the given MecanumDrive.
 */
public class TrajectoryFollowerState extends TimeoutSafetyState {
    protected Trajectory trajectory;

    private RoadrunnerMecanumDriveAdapter driveAdapter;

    private DriveConstraints baseConstraints;

    private MecanumConstraints constraints;

    private boolean initialized = false;

    // private MecanumPIDVAFollower follower;

    private HardwareMap hardwareMap;

    protected TrajectoryFollowerState(String name, Telemetry telemetry, long safetyTimeoutMillis,
                                      Trajectory trajectory,
                                      DriveConstraints baseConstraints,
                                      MecanumConstraints constraints,
                                      HardwareMap hardwareMap) {
        super(name, telemetry, safetyTimeoutMillis);

        if (trajectory == null) {
            throw new IllegalArgumentException("Trajectory required");
        }

        // FIXME: Check the rest of the arguments for correctness!

        this.trajectory = trajectory;
        this.baseConstraints = baseConstraints;

        this.constraints = constraints;
        this.hardwareMap = hardwareMap;
    }

    @Override
    public State doStuffAndGetNextState() {
        if (isTimedOut()) {
            Log.d(Constants.LOG_TAG, getName() + " timed out, moving to next state");

            return nextState;

        }

        /*
        if (!initialized) {
            // This is different than the hardware (bah) so we have to lazy init
            driveAdapter = new RoadrunnerMecanumDriveAdapter(hardwareMap);


            // TODO: Eventually all of this data should come from arguments -- we should fix that
            // the next robot we build that uses this concept.
            //
            // TODO: tune kV, kA, and kStatic in the following follower
            // then tune the PID coefficients after you verify the open loop response is roughly correct
            follower = new MecanumPIDVAFollower(
                    driveAdapter,
                    new PIDCoefficients(0, 0, 0),
                    new PIDCoefficients(0, 0, 0),
                    .01283,
                    0,
                    0);

            follower.followTrajectory(trajectory);
            initialized = true;
        }

        if (follower.isFollowing()) {
            Pose2d currentPose = driveAdapter.getPoseEstimate();

            Log.d(Constants.LOG_TAG, "Pose: " + currentPose.getX() + ", " + currentPose.getY()
                    + ", " + currentPose.getHeading());

            follower.update(currentPose);
            driveAdapter.updatePoseEstimate();

            return this;
        }

*/

        // stop moving
        driveAdapter.setMotorPowers(0, 0, 0, 0);

        return nextState;

    }

    @Override
    public void liveConfigure(DebouncedGamepadButtons buttons) {

    }
}
