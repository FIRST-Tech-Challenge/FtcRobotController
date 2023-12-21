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

package org.firstinspires.ftc.teamcode.Processors;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodParameters;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.List;

public abstract class MultiTfodProcessorParent implements VisionProcessor
{
    public static MultiTfodProcessorParent easyCreateWithDefaults() {
        return new Builder().build();
    }

    public static class Builder {
        private final TfodParameters.Builder builder = new TfodParameters.Builder();
        private List<MultiTfodProcessorChild> children = new ArrayList<>();

        /**
         * Set the name of the asset where the model is found.
         */
        public Builder setModelAssetName(String assetName) {
            builder.setModelAssetName(assetName);
            return this;
        }

        /**
         * Set the name of the file where the model is found.
         */
        public Builder setModelFileName(String fileName) {
            builder.setModelFileName(fileName);
            return this;
        }

        /**
         * Set the full ordered list of labels the model is trained to recognize.
         */
        public Builder setModelLabels(List<String> labels) {
            builder.setModelLabels(labels);
            return this;
        }
        public Builder setModelLabels(String[] labels) {
            builder.setModelLabels(labels);
            return this;
        }

        /**
         * Set whether the model is a TensorFlow2 model.
         */
        public Builder setIsModelTensorFlow2(boolean isModelTensorFlow2) {
            builder.setIsModelTensorFlow2(isModelTensorFlow2);
            return this;
        }

        /**
         * Set whether the model is quantized.
         */
        public Builder setIsModelQuantized(boolean isModelQuantized) {
            builder.setIsModelQuantized(isModelQuantized);
            return this;
        }

        /**
         * Set the size, in pixels, of images input to the network.
         */
        public Builder setModelInputSize(int inputSize) {
            builder.setModelInputSize(inputSize);
            return this;
        }

        /**
         * Set the aspect ratio for the images used when the model was created.
         */
        public Builder setModelAspectRatio(double modelAspectRatio) {
            builder.setModelAspectRatio(modelAspectRatio);
            return this;
        }

        /**
         * Set the number of executor threads to use. Each executor corresponds to one TensorFlow
         * Object Detector.
         */
        public Builder setNumExecutorThreads(int numExecutorThreads) {
            builder.setNumExecutorThreads(numExecutorThreads);
            return this;
        }

        /**
         * Set the number of threads to allow each individual TensorFlow object detector to use.
         */
        public Builder setNumDetectorThreads(int numDetectorThreads) {
            builder.setNumDetectorThreads(numDetectorThreads);
            return this;
        }

        /**
         * Set the maximum number of recognitions the network will return.
         */
        public Builder setMaxNumRecognitions(int maxNumRecognitions) {
            builder.setMaxNumRecognitions(maxNumRecognitions);
            return this;
        }

        /**
         * Set whether to use the tracker.
         */
        public Builder setUseObjectTracker(boolean useObjectTracker) {
            builder.setUseObjectTracker(useObjectTracker);
            return this;
        }

        /**
         * Set the maximum percentage of a box that can be overlapped by another box at recognition time.
         */
        public Builder setTrackerMaxOverlap(float trackerMaxOverlap) {
            builder.setTrackerMaxOverlap(trackerMaxOverlap);
            return this;
        }

        /**
         * Set the minimum size of an object that the tracker will track.
         */
        public Builder setTrackerMinSize(float trackerMinSize) {
            builder.setTrackerMinSize(trackerMinSize);
            return this;
        }

        /**
         * Allow replacement of the tracked box with new results if correlation has dropped below
         * trackerMarginalCorrelation.
         */
        public Builder setTrackerMarginalCorrelation(float trackerMarginalCorrelation) {
            builder.setTrackerMarginalCorrelation(trackerMarginalCorrelation);
            return this;
        }

        /**
         * Consider an object to be lost if correlation falls below trackerMinCorrelation.
         */
        public Builder setTrackerMinCorrelation(float trackerMinCorrelation) {
            builder.setTrackerMinCorrelation(trackerMinCorrelation);
            return this;
        }

        /**
        * Allows you to set stuff
        */
        public Builder addChild(MultiTfodProcessorChild child) {
            if (child !=null) this.children.add(child);
            return this;
        }

        /**
         * Returns a TfodProcessor object.
         */
        public MultiTfodProcessorParent build() {
            return new MultiTfodProcessorParentImpl(builder.build(), children);
        }
    }

    /**
     * Set the minimum confidence at which to keep recognitions.
     */
    public abstract void setMinResultConfidence(float minResultConfidence);

    /**
     * Sets the number of pixels to obscure on the left, top, right, and bottom edges of each image
     * passed to the TensorFlow object detector. The size of the images are not changed, but the
     * pixels in the margins are colored black.
     */
    public abstract void setClippingMargins(int left, int top, int right, int bottom);

    /**
     * Indicates that only the zoomed center area of each image will be passed to the TensorFlow
     * object detector. For no zooming, set magnification to 1.0.
     */
    public abstract void setZoom(double magnification);

    /**
     * Gets a list containing the latest recognitions, which may be stale.
     */
    public abstract List<Recognition> getRecognitions();

    /**
     * Gets a list containing recognitions that were detected since the last call to this method,
     * or null if no new recognitions are available. This is useful to avoid re-processing the same
     * recognitions multiple times.
     * @return a list containing fresh recognitions, or null.
     */
    public abstract List<Recognition> getFreshRecognitions();

    /**
     * Perform whatever cleanup is necessary to release all acquired resources.
     */
    public abstract void shutdown();
}
