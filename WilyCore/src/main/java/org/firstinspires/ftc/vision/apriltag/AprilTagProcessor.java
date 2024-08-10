/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.vision.apriltag;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.openftc.apriltag.AprilTagDetectorJNI;

import java.util.ArrayList;

public abstract class AprilTagProcessor implements VisionProcessor
{
    public static final int THREADS_DEFAULT = 3;

    public enum TagFamily
    {
        TAG_36h11(AprilTagDetectorJNI.TagFamily.TAG_36h11),
        TAG_25h9(AprilTagDetectorJNI.TagFamily.TAG_25h9),
        TAG_16h5(AprilTagDetectorJNI.TagFamily.TAG_16h5),
        TAG_standard41h12(AprilTagDetectorJNI.TagFamily.TAG_standard41h12);

        final AprilTagDetectorJNI.TagFamily ATLibTF;

        TagFamily(AprilTagDetectorJNI.TagFamily ATLibTF)
        {
            this.ATLibTF = ATLibTF;
        }
    }

    public static AprilTagProcessor easyCreateWithDefaults()
    {
        return new AprilTagProcessor.Builder().build();
    }

    public static class Builder
    {
        private double fx, fy, cx, cy;
        private TagFamily tagFamily = TagFamily.TAG_36h11;
        private AprilTagLibrary tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
        private DistanceUnit outputUnitsLength = DistanceUnit.INCH;
        private AngleUnit outputUnitsAngle = AngleUnit.DEGREES;
        private int threads = THREADS_DEFAULT;

        private boolean drawAxes = false;
        private boolean drawCube = false;
        private boolean drawOutline = true;
        private boolean drawTagId = true;

        /**
         * Set the camera calibration parameters (needed for accurate 6DOF pose unless the
         * SDK has a built in calibration for your camera)
         * @param fx see opencv 8 parameter camera model
         * @param fy see opencv 8 parameter camera model
         * @param cx see opencv 8 parameter camera model
         * @param cy see opencv 8 parameter camera model
         * @return the {@link Builder} object, to allow for method chaining
         */
        public Builder setLensIntrinsics(double fx, double fy, double cx, double cy)
        {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            return this;
        }

        /**
         * Set the tag family this detector will be used to detect (it can only be used
         * for one tag family at a time)
         * @param tagFamily the tag family this detector will be used to detect
         * @return the {@link Builder} object, to allow for method chaining
         */
        public Builder setTagFamily(TagFamily tagFamily)
        {
            this.tagFamily = tagFamily;
            return this;
        }

        /**
         * Inform the detector about known tags. The tag library is used to allow solving
         * for 6DOF pose, based on the physical size of the tag. Tags which are not in the
         * library will not have their pose solved for
         * @param tagLibrary a library of known tags for the detector to use when trying to solve pose
         * @return the {@link Builder} object, to allow for method chaining
         */
        public Builder setTagLibrary(AprilTagLibrary tagLibrary)
        {
            this.tagLibrary = tagLibrary;
            return this;
        }

        /**
         * Set the units you want translation and rotation data provided in, inside any
         * @param distanceUnit translational units
         * @param angleUnit rotational units
         * @return the {@link Builder} object, to allow for method chaining
         */
        public Builder setOutputUnits(DistanceUnit distanceUnit, AngleUnit angleUnit)
        {
            this.outputUnitsLength = distanceUnit;
            this.outputUnitsAngle = angleUnit;
            return this;
        }

        /**
         * Set whether to draw a 3D crosshair on the tag (what Vuforia did)
         * @param drawAxes whether to draw it
         * @return the {@link Builder} object, to allow for method chaining
         */
        public Builder setDrawAxes(boolean drawAxes)
        {
            this.drawAxes = drawAxes;
            return this;
        }

        /**
         * Set whether to draw a 3D cube projecting from the tag
         * @param drawCube whether to draw it lol
         * @return the {@link Builder} object, to allow for method chaining
         */
        public Builder setDrawCubeProjection(boolean drawCube)
        {
            this.drawCube = drawCube;
            return this;
        }

        /**
         * Set whether to draw a 2D outline around the tag detection
         * @param drawOutline whether to draw it
         * @return the {@link Builder} object, to allow for method chaining
         */
        public Builder setDrawTagOutline(boolean drawOutline)
        {
            this.drawOutline = drawOutline;
            return this;
        }

        /**
         * Set whether to annotate the tag detection with its ID
         * @param drawTagId whether to annotate the tag with its ID
         * @return the {@link Builder} object, to allow for method chaining
         */
        public Builder setDrawTagID(boolean drawTagId)
        {
            this.drawTagId = drawTagId;
            return this;
        }

        /**
         * Set the number of threads the tag detector should use
         * @param threads the number of threads the tag detector should use
         * @return the {@link Builder} object, to allow for method chaining
         */
        public Builder setNumThreads(int threads)
        {
            this.threads = threads;
            return this;
        }

        /**
         * Create a {@link VisionProcessor} object which may be attached to
         * a {@link org.firstinspires.ftc.vision.VisionPortal} using
         * {@link org.firstinspires.ftc.vision.VisionPortal.Builder#addProcessor(VisionProcessor)}
         * @return a {@link VisionProcessor} object
         */
        public AprilTagProcessor build()
        {
            if (tagLibrary == null)
            {
                throw new RuntimeException("Cannot create AprilTagProcessor without setting tag library!");
            }

            if (tagFamily == null)
            {
                throw new RuntimeException("Cannot create AprilTagProcessor without setting tag family!");
            }

            return new AprilTagProcessorImpl(
                    fx, fy, cx, cy,
                    outputUnitsLength, outputUnitsAngle, tagLibrary,
                    drawAxes, drawCube, drawOutline, drawTagId,
                    tagFamily, threads
            );
        }
    }

    /**
     * Set the detector decimation
     *
     * Higher decimation increases frame rate at the expense of reduced range
     *
     * @param decimation detector decimation
     */
    public abstract void setDecimation(float decimation);

    public enum PoseSolver
    {
        APRILTAG_BUILTIN(-1),
        OPENCV_ITERATIVE(0), // ### Calib3d.SOLVEPNP_ITERATIVE),
        OPENCV_SOLVEPNP_EPNP(1), // ### Calib3d.SOLVEPNP_EPNP),
        OPENCV_IPPE(2), // ### Calib3d.SOLVEPNP_IPPE),
        OPENCV_IPPE_SQUARE(3), // ### Calib3d.SOLVEPNP_IPPE_SQUARE),
        OPENCV_SQPNP(4); // ### Calib3d.SOLVEPNP_SQPNP);

        final int code;

        PoseSolver(int code)
        {
            this.code = code;
        }
    }

    /**
     * Specify the method used to calculate 6DOF pose from the tag corner positions once
     * found by the AprilTag algorithm
     * @param poseSolver the pose solver to use
     */
    public abstract void setPoseSolver(PoseSolver poseSolver);

    /**
     * Get the average time in milliseconds the currently set pose solver is taking
     * to converge on a solution PER TAG. Some pose solvers are much more expensive
     * than others...
     * @return average time to converge on a solution per tag in milliseconds
     */
    public abstract int getPerTagAvgPoseSolveTime();

    /**
     * Get a list containing the latest detections, which may be stale
     * i.e. the same as the last time you called this
     * @return a list containing the latest detections.
     */
    public abstract ArrayList<AprilTagDetection> getDetections();

    /**
     * Get a list containing detections that were detected SINCE THE PREVIOUS CALL to this method,
     * or NULL if no new detections are available. This is useful to avoid re-processing the same
     * detections multiple times.
     * @return a list containing fresh detections, or NULL.
     */
    public abstract ArrayList<AprilTagDetection> getFreshDetections();
}

