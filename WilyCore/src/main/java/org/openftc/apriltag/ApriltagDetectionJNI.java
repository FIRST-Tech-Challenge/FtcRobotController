/*
 * Copyright (c) 2021 OpenFTC Team
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

package org.openftc.apriltag;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.opencv.core.Point;

import java.util.ArrayList;

public class ApriltagDetectionJNI
{
    /**
     * Get the tag ID of a detection
     * @param ptr pointer to a detection, obtained from {@link #getDetectionPointers(long)}
     * @return the tag ID
     */
    public static native int getId(long ptr);

    /**
     * Get the hamming of a detection
     * @param ptr pointer to a detection, obtained from {@link #getDetectionPointers(long)}
     * @return the hamming value
     */
    public static native int getHamming(long ptr);

    /**
     * Get the decision margin of a detection
     * @param ptr pointer to a detection, obtained from {@link #getDetectionPointers(long)}
     * @return the decision margin of the detection
     */
    public static native float getDecisionMargin(long ptr);

    /**
     * Get the centerpoint of a detection
     * @param ptr pointer to a detection, obtained from {@link #getDetectionPointers(long)}
     * @return the centerpoint of the detection
     */
    public static native double[] getCenterpoint(long ptr);

    /**
     * Get the corners of a detection
     * @param ptr pointer to a detection, obtained from {@link #getDetectionPointers(long)}
     * @return the corners of the detection
     */
    public static native double[][] getCorners(long ptr);

    /**
     * Get the pose estimate for a detection
     * @param ptr pointer to a detection, obtained from {@link #getDetectionPointers(long)}
     * @param tagSize size of the tag in meters
     * @param fx lens intrinsics fx
     * @param fy lens intrinsics fy
     * @param cx lens intrinsics cx
     * @param cy lens intrinsics cy
     * @return pose estimate for the detection. 0-2 are translation XYZ, 3-5 are yaw, pitch, roll
     */
    public static native double[] getPoseEstimate(long ptr, double tagSize, double fx, double fy, double cx, double cy);

    /**
     * Get a pointer for each of the detections inside a list returned by {@link AprilTagDetectorJNI#runApriltagDetector(long, long, int, int)}
     * @param ptrZarray native pointer from {@link AprilTagDetectorJNI#runApriltagDetector(long, long, int, int)}
     * @return an array of native pointers to detections inside the list
     */
    public static native long[] getDetectionPointers(long ptrZarray);

    /**
     * Frees a list of detections obtained from {@link AprilTagDetectorJNI#runApriltagDetector(long, long, int, int)}
     * AND the detections themselves. (Thus, after calling this, any pointers you may have previously obtained
     * from {@link #getDetectionPointers(long)} are INVALID)
     * @param ptrDetections native pointer to a list of detections
     */
    public static native void freeDetectionList(long ptrDetections);

    /**
     * Creates a nice decoupled java representation of the detections in the native detection list
     * @param ptrDetections native pointer from {@link AprilTagDetectorJNI#runApriltagDetector(long, long, int, int)}
     * @param tagSize size of the tag in meters
     * @param fx lens intrinsics fx
     * @param fy lens intrinsics fy
     * @param cx lens intrinsics cx
     * @param cy lens intrinsics cy
     * @return a nice decoupled java representation of the detections in the native detection list
     */
    public static ArrayList<AprilTagDetection> getDetections(long ptrDetections, double tagSize, double fx, double fy, double cx, double cy)
    {
        long[] detectionPointers = getDetectionPointers(ptrDetections);
        ArrayList<AprilTagDetection> detections = new ArrayList<>(detectionPointers.length);

        for(long ptrDetection : detectionPointers)
        {
            AprilTagDetection detection = new AprilTagDetection();
            detection.id = getId(ptrDetection);
            detection.hamming = getHamming(ptrDetection);
            detection.decisionMargin = getDecisionMargin(ptrDetection);
            double[] center = getCenterpoint(ptrDetection);
            detection.center = new Point(center[0], center[1]);
            double[][] corners = getCorners(ptrDetection);

            detection.corners = new Point[4];
            for(int p = 0; p < 4; p++)
            {
                detection.corners[p] = new Point(corners[p][0], corners[p][1]);
            }

            detection.pose = new AprilTagPose();
            double[] pose = getPoseEstimate(ptrDetection, tagSize, fx, fy, cx, cy);
            detection.pose.x = pose[0];
            detection.pose.y = pose[1];
            detection.pose.z = pose[2];

            float[] rotMtxVals = new float[3*3];

            for (int i = 0; i < 9; i++)
            {
                rotMtxVals[i] = (float) pose[3 + i];
            }

            detection.pose.R = new GeneralMatrixF(3, 3, rotMtxVals);

            detections.add(detection);
        }

        return detections;
    }

    static
    {
        System.loadLibrary("apriltag");
    }
}
