/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
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
*/

package com.hfrobots.tnt.season1920.opencv;

import android.util.Log;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class DetectionZone implements Comparable<DetectionZone> {
    private double totalBlackArea = 0.0D;

    final int zoneMinX;

    final int zoneMaxX;

    private final String zoneName;

    public DetectionZone(final String zoneName, final int zoneMinX, final int zoneMaxX) {
        // prevent mistakes due to swap of values
        if (zoneMaxX < zoneMinX) {
            this.zoneMinX = zoneMaxX;
            this.zoneMaxX = zoneMinX;
        } else {
            this.zoneMinX = zoneMinX;
            this.zoneMaxX = zoneMaxX;
        }

        this.zoneName = zoneName;
    }

    double maxContourSize = Double.MIN_VALUE;

    public void accumulateBlackContourArea(MatOfPoint contour, Rect boundingRectangle) {
        double contourArea = Imgproc.contourArea(contour);

        if (contourArea > maxContourSize) {
            maxContourSize = contourArea;

            Log.d(LOG_TAG, "Max contour area now:" + maxContourSize);
        }

        if (contourArea <= 1000) {
            return;
        }

        int boundingRectangleXStart = boundingRectangle.x;

        int boundingRectangleWidth = boundingRectangle.width;

        int boundingRectangleXEnd = boundingRectangleXStart + boundingRectangleWidth;

        if (
            // (1) X values of bounding rectangle completely inside detection zone

                withinZoneEndExclusive(boundingRectangleXStart) &&
                withinZoneEndInclusive(boundingRectangleXEnd)) {

            totalBlackArea += boundingRectangleWidth;
        } else if (
            // (2) X value starts within bounding zone, but ends outside detection zone

                withinZoneEndExclusive(boundingRectangleXStart) &&
                outsideOfZoneEndExclusive(boundingRectangleXEnd)) {


            double amountInZone = Math.abs(zoneMaxX - boundingRectangleXStart);

            totalBlackArea += amountInZone;
        } else if (
            // (3) X value starts before detection zone, ends inside detection zone

                beforeZoneBegins(boundingRectangleXStart) &&
                withinZoneEndInclusive(boundingRectangleXEnd)) {


            double amountInZone = Math.abs(boundingRectangleXEnd - zoneMinX);

            totalBlackArea = amountInZone;
        } else if (
            // (4) X value starts before bounding zone, ends outside detection zone

                beforeZoneBegins(boundingRectangleXStart) &&
                outsideOfZoneEndExclusive(boundingRectangleXEnd)) {

            int amountInZone = Math.abs(zoneMaxX - zoneMinX);

            totalBlackArea += amountInZone;
        }

    }


    public String getZoneName() {
        return zoneName;
    }

    public double getTotalBlackArea() {
        return totalBlackArea;
    }

    // the point 'x' begins before zoneMinX
    private boolean beforeZoneBegins(int x) {
        return x < zoneMinX;
    }

    // the point 'x' is somewhere between zone-x min and max
    private boolean withinZoneEndExclusive(int x) {
        return x >= zoneMinX && x < zoneMaxX;
    }

    private boolean withinZoneEndInclusive(int x) {
        return x >= zoneMinX && x <= zoneMaxX;
    }

    private boolean outsideOfZoneEndExclusive(int x) {
        return x > zoneMaxX;
    }

    @Override
    public int compareTo(DetectionZone other) {
        double otherTotalBlackArea = other.totalBlackArea;


        // return negative integer, zero, or a positive integer as this object is less than, equal
        // to, or greater than the specified object.
        // if the two area are equal a zero will be returned.
        // if the "other" area is greater a negative value will be returned
        // if this area is greater a positive value will be returned

        return (int)(totalBlackArea-otherTotalBlackArea);
    }
}
