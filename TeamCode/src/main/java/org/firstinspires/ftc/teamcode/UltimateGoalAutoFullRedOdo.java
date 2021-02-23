/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.HelperClasses.WayPoint;

/**
 * Created by 7592 RoarBots
 */
@Autonomous(name="Full Red Odo", group ="Ultimate Goal Red", preselectTeleOp = "UltimateGoal: TeleOp")
public class UltimateGoalAutoFullRedOdo extends UltimateGoalAutoFullOdo
{
    protected static final double REGULAR_SPEED = 1.0;
    protected static final double SLOW_SPEED = 0.25;
    @Override
    public void setAutoWayPoints() {
        // Robot starting location
        startLocation = new WayPoint(149.7584, 22.86, Math.toRadians(90.0), 0.0);

        // These waypoints maneuver the robot around the starting ring stack.
        aroundStartingStack1 = new WayPoint(89.47404, 89.47404, Math.toRadians(90.0), REGULAR_SPEED);
        aroundStartingStack2 = new WayPoint(89.47404, 149.7584, Math.toRadians(93.0), REGULAR_SPEED);

        // The powershots
        switch(startShootingStyle) {
            case STRAFE_STOP:
            case HIGH_GOAL:
                powerShotFirst = new WayPoint(69.77404, 149.7584, Math.toRadians(93.0), REGULAR_SPEED);
                UltimateGoalRobot.powerShotLeft = powerShotFirst;
                powerShotSecond = new WayPoint(98.77404, 149.7584, Math.toRadians(95.0), REGULAR_SPEED);
                UltimateGoalRobot.powerShotCenter = powerShotSecond;
                powerShotThird = new WayPoint(117.77404, 149.7584, Math.toRadians(95.0), REGULAR_SPEED);
                UltimateGoalRobot.powerShotRight = powerShotThird;
                break;
            case STRAFE_THROUGH:
                powerShotFirst = new WayPoint(69.77404, 149.7584, Math.toRadians(93.0), SLOW_SPEED);
                UltimateGoalRobot.powerShotLeft = powerShotFirst;
                powerShotSecond = new WayPoint(98.77404, 149.7584, Math.toRadians(95.0), SLOW_SPEED);
                UltimateGoalRobot.powerShotCenter = powerShotSecond;
                powerShotThird = new WayPoint(117.77404, 149.7584, Math.toRadians(95.0), SLOW_SPEED);
                UltimateGoalRobot.powerShotRight = powerShotThird;
                break;
            case ROTATE:
                powerShotFirst = new WayPoint(98.77404, 149.7584, Math.toRadians(93.0), REGULAR_SPEED);
                UltimateGoalRobot.powerShotLeft = powerShotFirst;
                powerShotSecond = new WayPoint(98.77404, 149.7584, Math.toRadians(95.0), REGULAR_SPEED);
                UltimateGoalRobot.powerShotCenter = powerShotSecond;
                powerShotThird = new WayPoint(98.77404, 149.7584, Math.toRadians(97.0), REGULAR_SPEED);
                UltimateGoalRobot.powerShotRight = powerShotThird;
                break;
        }

        // starting stack line up
        beforeStack = new WayPoint(149.7584, 195.44792, Math.toRadians(270.0), REGULAR_SPEED);
        collectStack = new WayPoint(149.7584, 135.44792, Math.toRadians(270.0), REGULAR_SPEED);

        // Collect the starting ring stack.
        collectStartingStack = new WayPoint(126.8984, 97.70872, Math.toRadians(85.0), 0.5);

        // Pickup the second wobble goal.
        wobble2PickupLineup = new WayPoint(175.0, 110.0, Math.toRadians(30.0), 0.25);
        wobble2Pickup = new WayPoint(185.0, 92.0, Math.toRadians(30.0), REGULAR_SPEED);

        // Shoot the collected rings in the high goal.
        highGoal = new WayPoint(164.35324, 149.7584, Math.toRadians(95.0), REGULAR_SPEED);
        UltimateGoalRobot.highGoal = highGoal;

        park = new WayPoint(149.7584, 195.44792, Math.toRadians(135.0), REGULAR_SPEED);
    }

    // Sets all the route points for executing the autonomous.
    @Override
    public void setRandomizationPosition(int position) {

        // The location specific collection values.
        switch(position) {
            case 1:
                // To deposit the first wobble goal, set by vision.
                targetZone1 = new WayPoint(166.25824, 180.85308, Math.toRadians(135.0), REGULAR_SPEED);

                // Deliver the second wobble goal.
                targetZone2 = new WayPoint(195.44792, 195.44792, Math.toRadians(135.0), REGULAR_SPEED);
                break;
            case 2:
                // To deposit the first wobble goal, set by vision.
                targetZone1 = new WayPoint(120.56872, 257.13744, Math.toRadians(135.0), REGULAR_SPEED);

                // Deliver the second wobble goal.
                targetZone2 = new WayPoint(120.56872, 237.73228, Math.toRadians(135.0), REGULAR_SPEED);
                break;
            case 3:
                // To deposit the first wobble goal, set by vision.
                targetZone1 = new WayPoint(190.85308, 306.4218, Math.toRadians(135.0), REGULAR_SPEED);

                // Deliver the second wobble goal.
                targetZone2 = new WayPoint(180.85308, 301.4218, Math.toRadians(135.0), REGULAR_SPEED);
                break;
        }
    }
}