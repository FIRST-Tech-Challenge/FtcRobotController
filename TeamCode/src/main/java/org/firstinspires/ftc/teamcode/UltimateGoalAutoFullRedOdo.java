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
    protected static final double SLOW_SPEED = 0.30;
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
//                powerShotFirst = new WayPoint(82.77404, 149.7584, Math.toRadians(95.0), REGULAR_SPEED);
//                powerShotSecond = new WayPoint(100.27404, 149.7584, Math.toRadians(95.0), REGULAR_SPEED);
//                powerShotThird = new WayPoint(122.77404, 149.7584, Math.toRadians(95.0), REGULAR_SPEED);
                powerShotFirst = new WayPoint(90.77404, 149.7584, Math.toRadians(91.25), 1.0);
                powerShotSecond = new WayPoint(90.77404, 149.7584, Math.toRadians(96.25), 1.0);
                powerShotThird = new WayPoint(90.77404, 149.7584, Math.toRadians(87.25), 1.0);
                beforeQuadStack = new WayPoint(152.865, 74.46187, Math.toRadians(71.43), SLOW_SPEED);
                collectQuadStack = new WayPoint(165.2126, 122.607, Math.toRadians(69.946), REGULAR_SPEED);
                collectQuadStraggler = new WayPoint(165.2126, 132.607, Math.toRadians(69.946), SLOW_SPEED);
                quadHighGoalCollecting = new WayPoint(175.673, 118.555, Math.toRadians(99.895), REGULAR_SPEED);
                quadHighGoalFinal = new WayPoint(175.673, 138.555, Math.toRadians(102.895), REGULAR_SPEED);
                quadSecondWobbleStart = new WayPoint(190.0, 150.0, Math.toRadians(45.0), REGULAR_SPEED);
                break;
            case STRAFE_THROUGH:
                powerShotFirst = new WayPoint(69.77404, 149.7584, Math.toRadians(93.0), REGULAR_SPEED);
                powerShotSecond = new WayPoint(90.77404, 149.7584, Math.toRadians(95.0), SLOW_SPEED);
                powerShotThird = new WayPoint(110.77404, 149.7584, Math.toRadians(95.0), SLOW_SPEED);
                break;
            case ROTATE:
                powerShotFirst = new WayPoint(98.77404, 149.7584, Math.toRadians(91.0), REGULAR_SPEED);
                powerShotSecond = new WayPoint(98.77404, 149.7584, Math.toRadians(95.0), REGULAR_SPEED);
                powerShotThird = new WayPoint(98.77404, 149.7584, Math.toRadians(98.0), REGULAR_SPEED);
                break;
        }

        // starting stack line up
//        beforeStack = new WayPoint(149.7584, 195.44792, Math.toRadians(270.0), SLOW_SPEED);
//        collectStack = new WayPoint(154.7584, 125.44792, Math.toRadians(270.0), SLOW_SPEED);
        beforeStack = new WayPoint(145.7584, 195.44792, Math.toRadians(270.0), SLOW_SPEED);
        collectStack = new WayPoint(150.7584, 125.44792, Math.toRadians(270.0), SLOW_SPEED);

        // Shoot the collected rings in the high goal.
        bumpStack = new WayPoint(167.35324, 139.7584, Math.toRadians(95.0), SLOW_SPEED);
        highGoal = new WayPoint(167.35324, 149.7584, Math.toRadians(95.0), REGULAR_SPEED);
        longShotHighGoal = new WayPoint(147.9975, 57.9297, Math.toRadians(96.0), REGULAR_SPEED);

        park = new WayPoint(149.7584, 195.44792, Math.toRadians(135.0), REGULAR_SPEED);
    }

    // Sets all the route points for executing the autonomous.
    @Override
    public void setRandomizationPosition(int position) {

        // The location specific collection values.
        switch(position) {
            case 1:
                startShootingStyle = StartShotStyle.STRAFE_STOP;
                // To deposit the first wobble goal, set by vision.
                targetZone1 = new WayPoint(191.25824, 185.85308, Math.toRadians(135.0), REGULAR_SPEED);

                // Deliver the second wobble goal.
                targetZone2 = new WayPoint(175.44792, 180.44792, Math.toRadians(135.0), REGULAR_SPEED);

                // Pickup the second wobble goal.
                wobble2PickupLineup = new WayPoint(165.0, 115.0, Math.toRadians(30.0), SLOW_SPEED);
                wobble2Pickup = new WayPoint(185.0, 90.0, Math.toRadians(30.0), REGULAR_SPEED);
                break;
            case 2:
                startShootingStyle = StartShotStyle.STRAFE_STOP;
                // To deposit the first wobble goal, set by vision.
                targetZone1 = new WayPoint(120.56872, 257.13744, Math.toRadians(135.0), REGULAR_SPEED);

                // Deliver the second wobble goal.
                targetZone2 = new WayPoint(120.56872, 237.73228, Math.toRadians(135.0), REGULAR_SPEED);

                // Pickup the second wobble goal.
                wobble2PickupLineup = new WayPoint(165.0, 115.0, Math.toRadians(30.0), SLOW_SPEED);
                wobble2Pickup = new WayPoint(180.0, 88.0, Math.toRadians(30.0), REGULAR_SPEED);
                break;
            case 3:
                startShootingStyle = StartShotStyle.HIGH_GOAL;
                // To deposit the first wobble goal, set by vision.
                targetZone1 = new WayPoint(194.85308, 305.4218, Math.toRadians(145.0), REGULAR_SPEED);
                wobblePullAway = new WayPoint(174.85308, 281.4218, Math.toRadians(145.0), REGULAR_SPEED);
                beforeStack = new WayPoint(200.0, 145.0, Math.toRadians(90.0), REGULAR_SPEED);

                // Pickup the second wobble goal.
                wobble2PickupLineup = new WayPoint(190.0, 130.0, Math.toRadians(45.0), SLOW_SPEED);
                wobble2Pickup = new WayPoint(190.0, 90.0, Math.toRadians(45.0), REGULAR_SPEED);

                // Deliver the second wobble goal.
                targetZone2 = new WayPoint(180.85308, 296.4218, Math.toRadians(135.0), REGULAR_SPEED);
                break;
        }
    }
}