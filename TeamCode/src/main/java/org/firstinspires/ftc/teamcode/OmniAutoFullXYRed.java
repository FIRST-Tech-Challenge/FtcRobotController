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

import org.opencv.core.Point;
import org.firstinspires.ftc.teamcode.HelperClasses.WayPoint;

/**
 * Created by 12090 STEM Punk
 */
//@Autonomous(name="Full Red", group ="Red")
public class OmniAutoFullXYRed extends OmniAutoFullXY
{
    // Sets the points in the image to detect the skystone.
    @Override
    public void setVisionPoints() {
        sub1PointA = new Point(174, 239); // Stone4, Position 1
        sub1PointB = new Point(184, 249);
        sub2PointA = new Point(174, 174); // Stone5, Position 2
        sub2PointB = new Point(184, 184);
        sub3PointA = new Point(174, 99);  // Stone6, Position 3
        sub3PointB = new Point(184, 109);
    }

    protected double skystoneX = 271.5463;
    protected double skystone1Y = 61.66;
    protected double skystone2Y = skystone1Y + 20.32;
    protected double skystone3Y = skystone2Y + 20.32;
    protected double skystone4Y = skystone3Y + 20.32;
    protected double skystone5Y = skystone4Y + 20.32;
    protected double skystone6Y = skystone5Y + 20.32;
    protected double runLaneX = 266.5831;

    protected double attackAngle = Math.toRadians(225.0);
    protected double runAngle = Math.toRadians(270.0);

    @Override
    public void setAutoWayPoints() {
        // Robot starting location
        startLocation = new WayPoint(335.915, 83.14436, Math.toRadians(180.0), 0.0);

        // small pull away from wall to rotate robot without hitting.
        distanceFromWall = new WayPoint(325.915, 83.14436, Math.toRadians(180.0), 0.5);

        // Get the robot under the bridge to do foundation
        buildSiteUnderBridge = new WayPoint(runLaneX, 235.9901, runAngle, 1.0);

        snuggleFoundation = new WayPoint(238.0663, 315.755, Math.toRadians(360.0), 0.3);
        grabFoundation = new WayPoint(232.0663, 315.755, Math.toRadians(360), 0.1);
        pullFoundation = new WayPoint(275.3956, 281.206, Math.toRadians(300), 1.0);
        pushFoundation = new WayPoint(275.3956, 277.785, runAngle, 0.7);

        // Might be able to use buildSiteUnderBridge for this.
        buildSiteDodgingPartner = new WayPoint(runLaneX + 5, 271.206, runAngle, 0.7);

        // Same as buildSiteReadyToRun but facing towards center of the field.
        buildSiteEjectingStone = new WayPoint(runLaneX, 224.9901, Math.toRadians(180.0), 0.7);

        // Need to tweak this down so the robot is parked while waiting with
        // just intake wheels.  Calculated is 225.9901, start at 235.9901 for safety
        buildSiteReadyToRun = new WayPoint(runLaneX, 224.9901, runAngle, 0.7);
        quarryUnderBridge = new WayPoint(runLaneX, 185.26, runAngle, 0.7);
        foundationDeposit = new WayPoint(runLaneX, 297.785, runAngle, 1.0);
        park = new WayPoint(runLaneX, 204.3875, runAngle, 1.0);
    }

    // Sets all the route points for executing the autonomous.
    @Override
    public void setSkystoneValues(int position) {
        // The location specific skystone collection values.
        switch(position) {
            case 1:
                // Skystone position 1 specific coordinates
                positionToGrabSkystone1 = new WayPoint(skystoneX, skystone1Y, attackAngle, 1.0);
                grabSkystone1 = new WayPoint(skystoneX - 20.0, skystone1Y - 20.0, attackAngle, 1.0);
                pullBackSkystone1 = new WayPoint(runLaneX, skystone1Y - 20.0, attackAngle, 0.5);
                // Skystone position 4 specific coordinates
                positionToGrabSkystone2 = new WayPoint(skystoneX, skystone4Y, attackAngle, 1.0);
                grabSkystone2 = new WayPoint(skystoneX - 20.0, skystone4Y - 20.0, attackAngle, 1.0);
                pullBackSkystone2 = new WayPoint(runLaneX, skystone4Y - 20.0, runAngle, 0.5);
                // Stretch goals
                positionToGrabMundanestone1 = new WayPoint(skystoneX, skystone3Y, attackAngle, 1.0);
                grabMundanestone1 = new WayPoint(skystoneX - 20.0, skystone3Y - 20.0, attackAngle, 1.0);
                pullBackMundanestone1 = new WayPoint(runLaneX, skystone3Y - 20.0, runAngle, 0.5);
                break;
            case 2:
                // Skystone position 2 specific coordinates
                positionToGrabSkystone1 = new WayPoint(skystoneX, skystone2Y, attackAngle, 1.0);
                grabSkystone1 = new WayPoint(skystoneX - 20.0, skystone2Y - 20.0, attackAngle, 1.0);
                pullBackSkystone1 = new WayPoint(runLaneX, skystone2Y - 20.0, runAngle, 0.5);
                // Skystone position 5 specific coordinates
                positionToGrabSkystone2 = new WayPoint(skystoneX, skystone5Y, attackAngle, 1.0);
                grabSkystone2 = new WayPoint(skystoneX - 20.0, skystone5Y - 20.0, attackAngle, 1.0);
                pullBackSkystone2 = new WayPoint(runLaneX, skystone5Y - 20.0, runAngle, 0.5);
                // Stretch goals
                positionToGrabMundanestone1 = new WayPoint(skystoneX - 5.0, skystone4Y + 5.0, attackAngle, 1.0);
                grabMundanestone1 = new WayPoint(skystoneX - 15.0, skystone4Y - 15.0, attackAngle, 1.0);
                pullBackMundanestone1 = new WayPoint(runLaneX, skystone4Y - 20.0, runAngle, 0.5);
                break;
            case 3:
                // Skystone position 3 specific coordinates
                positionToGrabSkystone1 = new WayPoint(skystoneX, skystone3Y, attackAngle, 1.0);
                grabSkystone1 = new WayPoint(skystoneX - 20.0, skystone3Y - 20.0, attackAngle, 1.0);
                pullBackSkystone1 = new WayPoint(runLaneX, skystone3Y - 20.0, runAngle, 0.5);
                // Skystone position 6 specific coordinates
                positionToGrabSkystone2 = new WayPoint(skystoneX, skystone6Y, attackAngle, 1.0);
                grabSkystone2 = new WayPoint(skystoneX - 20.0, skystone6Y - 20.0, attackAngle, 1.0);
                pullBackSkystone2 = new WayPoint(runLaneX, skystone6Y - 30.0, runAngle, 0.5);
                // Stretch goals
                positionToGrabMundanestone1 = new WayPoint(skystoneX - 10.0, skystone1Y + 10.0, attackAngle, 1.0);
                grabMundanestone1 = new WayPoint(skystoneX - 10.0, skystone1Y - 10.0, attackAngle, 1.0);
                pullBackMundanestone1 = new WayPoint(runLaneX, skystone1Y - 20.0, attackAngle, 0.5);
                break;
        }

    }
}