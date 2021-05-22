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
import org.opencv.core.Point;

/**
 * Created by 7592 RoarBots
 */
//@Autonomous(name="Full Red", group ="Ultimate Goal Red", preselectTeleOp = "UltimateGoal: TeleOp")
public class UltimateGoalAutoFullRed extends UltimateGoalAutoFull
{
    protected static final double REGULAR_SPEED = 0.5;
    @Override
    public void setAutoWayPoints() {
        // Robot starting location
        startLocation = new WayPoint(180.85308, 22.86, Math.toRadians(90.0), 0.0);

        // These waypoints maneuver the robot around the starting ring stack.
        aroundStartingStack1 = new WayPoint(194.49542, 89.47404, Math.toRadians(90.0), REGULAR_SPEED);
        aroundStartingStack2 = new WayPoint(194.49542, 149.7584, Math.toRadians(90.0), REGULAR_SPEED);

        // The powershots
        powerShotFirst = new WayPoint(104.06888, 178.94808, Math.toRadians(95.0), REGULAR_SPEED);
        powerShotSecond = new WayPoint(89.47404, 178.94808, Math.toRadians(95.0), REGULAR_SPEED);
        powerShotThird = new WayPoint(74.8792, 178.94808, Math.toRadians(95.0), REGULAR_SPEED);

        // Pickup the second wobble goal.
        wobble2Pickup = new WayPoint(97.70872, 83.14436, Math.toRadians(85.0), REGULAR_SPEED);

        // Collect the starting ring stack.
        collectStartingStack = new WayPoint(126.8984, 97.70872, Math.toRadians(85.0), REGULAR_SPEED);

        // Shoot the collected rings in the high goal.
        highGoal = new WayPoint(164.35324, 187.18276, Math.toRadians(95.0), REGULAR_SPEED);

        park = new WayPoint(149.7584, 210.04276, Math.toRadians(110.0), REGULAR_SPEED);
    }

    // Sets all the route points for executing the autonomous.
    @Override
    public void setRandomizationPosition(int position) {
        // The location specific collection values.
        switch(position) {
            case 1:
                // To deposit the first wobble goal, set by vision.
                targetZone1 = new WayPoint(210.04276, 210.04276, Math.toRadians(120.0), REGULAR_SPEED);

                // Deliver the second wobble goal.
                targetZone2 = new WayPoint(195.44792, 195.44792, Math.toRadians(120.0), REGULAR_SPEED);
                break;
            case 2:
                // To deposit the first wobble goal, set by vision.
                targetZone1 = new WayPoint(149.7584, 270.32712, Math.toRadians(120.0), REGULAR_SPEED);

                // Deliver the second wobble goal.
                targetZone2 = new WayPoint(135.16356, 255.73228, Math.toRadians(120.0), REGULAR_SPEED);
                break;
            case 3:
                // To deposit the first wobble goal, set by vision.
                targetZone1 = new WayPoint(210.04276, 330.61148, Math.toRadians(120.0), REGULAR_SPEED);

                // Deliver the second wobble goal.
                targetZone2 = new WayPoint(195.44792, 316.01664, Math.toRadians(120.0), REGULAR_SPEED);
                break;
        }
    }
}