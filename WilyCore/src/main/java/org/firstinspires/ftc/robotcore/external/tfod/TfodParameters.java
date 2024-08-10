/*
 * Copyright (C) 2018 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.firstinspires.ftc.robotcore.external.tfod;

import androidx.annotation.Nullable;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Parameters which can be used to configure the performance and behavior of TFObjectDetector.
 *
 * <p>This class is intended to be instantiated through the Builder object, which helps provide
 * reasonable defaults for all of the parameter values. Note that changing the parameter values away
 * from the defaults may in some cases make little to no performance impact, while others may
 * dramatically increase the CPU load of the system.
 *
 * @author Vasu Agrawal
 * @author Liz Looney
 */
public class TfodParameters {
    public static class CurrentGame {
        public static final String MODEL_ASSET = "CenterStage.tflite";
        public static final String[] LABELS = {
                "Pixel"
        };
        public static final boolean IS_MODEL_TENSOR_FLOW_2 = true;
        public static final boolean IS_MODEL_QUANTIZED = true;
        public static final int MODEL_INPUT_SIZE = 300;
        public static final double MODEL_ASPECT_RATIO = 16.0/9.0;
    }

    /**
     * The name of the asset where the model is found.
     */
    @Nullable
    public final String modelAssetName;

    /**
     * The name of the file where the model is found.
     */
    @Nullable
    public final String modelFileName;

    /**
     * The full ordered list of labels the model is trained to recognize.
     */
    public final List<String> modelLabels; // not modifyable

    /**
     * Whether the model is a TensorFlow2 model.
     */
    public final boolean isModelTensorFlow2;

    /**
     * Whether the model is quantized.
     */
    public final boolean isModelQuantized;

    /**
     * The size in pixels of images input to the network, assuming the input is a square.
     *
     * <p>For example, if the network input size is 300 x 300, this parameter would be 300.
     */
    public final int modelInputSize;

    /**
     * The aspect ratio for the images used when the model was created.
     */
    public final double modelAspectRatio;

    /**
     * The number of executor threads to use. Each executor cooresponds to one TensorFlow
     * Object Detector.
     */
    public final int numExecutorThreads;

    /**
     * The number of threads to allow each individual TensorFlow object detector to use.
     */
    public final int numDetectorThreads;

    /**
     * The maximum number of recognitions the network will return.
     */
    public final int maxNumRecognitions;

    /**
     * Whether to use the tracker.
     */
    public final boolean useObjectTracker;

    // TODO(vasuagrawal): Figure out what these things are.
    // Not quite sure what these parameters do, but they were in the tracker and can be adjusted.
    // Any comments here are copied from the original source. See MultiBoxTracker.
    /**
     * Maximum percentage of a box that can be overlapped by another box at recognition time.
     *
     * <p>Otherwise, the lower scored box (new or old) will be removed.
     */
    public final float trackerMaxOverlap;

    public final float trackerMinSize;

    /**
     * Allow replacement of the tracked box with new results if correlation has dropped below this.
     */
    public final float trackerMarginalCorrelation;

    /**
     * Consider object to be lost if correlation falls below this threshold.
     */
    public final float trackerMinCorrelation;

    // Private constructor to force clients to use the Builder and get proper argument verification
    private TfodParameters(Builder builder) {
        if (builder.modelAssetName == null && builder.modelFileName == null) {
            throw new IllegalStateException("modelAssetName and modelFileName cannot both be null");
        }
        if (builder.modelAssetName != null && builder.modelFileName != null) {
            throw new IllegalStateException("modelAssetName and modelFileName cannot both be non-null");
        }
        modelAssetName = builder.modelAssetName;
        modelFileName = builder.modelFileName;
        modelLabels = Collections.unmodifiableList(builder.modelLabels);
        isModelTensorFlow2 = builder.isModelTensorFlow2;
        isModelQuantized = builder.isModelQuantized;
        modelInputSize = builder.modelInputSize;
        modelAspectRatio = builder.modelAspectRatio;

        numExecutorThreads = builder.numExecutorThreads;
        numDetectorThreads = builder.numDetectorThreads;
        maxNumRecognitions = builder.maxNumRecognitions;

        useObjectTracker = builder.useObjectTracker;
        trackerMaxOverlap = builder.trackerMaxOverlap;
        trackerMinSize = builder.trackerMinSize;
        trackerMarginalCorrelation = builder.trackerMarginalCorrelation;
        trackerMinCorrelation = builder.trackerMinCorrelation;
    }

    public static class Builder {
        // Default to using the model for the current game.
        private String modelAssetName = CurrentGame.MODEL_ASSET;
        private String modelFileName;
        private final List<String> modelLabels = new ArrayList<>();
        private boolean isModelTensorFlow2 = CurrentGame.IS_MODEL_TENSOR_FLOW_2;
        private boolean isModelQuantized = CurrentGame.IS_MODEL_QUANTIZED;
        private int modelInputSize = CurrentGame.MODEL_INPUT_SIZE;
        private double modelAspectRatio = CurrentGame.MODEL_ASPECT_RATIO;

        private int numExecutorThreads = 2;
        private int numDetectorThreads = 1;

        private int maxNumRecognitions = 10;

        private boolean useObjectTracker = true;
        private float trackerMaxOverlap = 0.2f;
        private float trackerMinSize = 16.0f;
        private float trackerMarginalCorrelation = 0.75f;
        private float trackerMinCorrelation = 0.3f;

        public Builder() {
            setModelLabels(Arrays.asList(CurrentGame.LABELS));
        }

        public Builder setModelAssetName(String modelAssetName) {
// ###            // Here we just check that the asset exists so we can give an error message.
// ###            try {
// ###                AssetManager assetManager = AppUtil.getDefContext().getAssets();
// ###                AssetFileDescriptor afd = assetManager.openFd(modelAssetName);
// ###                afd.close();
// ###            } catch (IOException e) {
// ###                throw new IllegalStateException("Could not find asset named " + modelAssetName);
// ###            }
            this.modelAssetName = modelAssetName;
            this.modelFileName = null;
            return this;
        }

        public Builder setModelFileName(String modelFileName) {
            // Here we just check that the file exists so we can give an error message.
// ###             File file = new File(TFLITE_MODELS_DIR, modelFileName);
// ###             if (!file.exists()) {
// ###                 file = new File(modelFileName);
// ###                 if (!file.exists()) {
// ###                     throw new IllegalStateException("Could not find file named " + modelFileName);
// ###                 }
// ###             }
            this.modelAssetName = null;
            this.modelFileName = modelFileName;
            return this;
        }

        public Builder setModelLabels(List<String> modelLabels) {
            this.modelLabels.clear();
            for (String label : modelLabels) {
                this.modelLabels.add(label);
            }
            return this;
        }

        public Builder setModelLabels(String[] modelLabels) {
            this.modelLabels.clear();
            for (String label : modelLabels) {
                this.modelLabels.add(label);
            }
            return this;
        }

        public Builder setIsModelTensorFlow2(boolean isModelTensorFlow2) {
            this.isModelTensorFlow2 = isModelTensorFlow2;
            return this;
        }

        public Builder setIsModelQuantized(boolean isModelQuantized) {
            this.isModelQuantized = isModelQuantized;
            return this;
        }

        public Builder setModelInputSize(int modelInputSize) {
            this.modelInputSize = modelInputSize;
            return this;
        }

        public Builder setModelAspectRatio(double modelAspectRatio) {
            this.modelAspectRatio = modelAspectRatio;
            return this;
        }

        public Builder setNumExecutorThreads(int numExecutorThreads) {
            if (numExecutorThreads <= 0) {
                throw new IllegalArgumentException("Must have at least 1 executor thread");
            }
            this.numExecutorThreads = numExecutorThreads;
            return this;
        }

        public Builder setNumDetectorThreads(int numDetectorThreads) {
            if (numDetectorThreads <= 0) {
                throw new IllegalArgumentException("Must have at least 1 thread per detector");
            }
            this.numDetectorThreads = numDetectorThreads;
            return this;
        }

        public Builder setMaxNumRecognitions(int maxNumRecognitions) {
            if (maxNumRecognitions <= 0) {
                throw new IllegalArgumentException("maxNumRecognitions must be at least 1");
            }
            this.maxNumRecognitions = maxNumRecognitions;
            return this;
        }

        public Builder setUseObjectTracker(boolean useObjectTracker) {
            this.useObjectTracker = useObjectTracker;
            return this;
        }

        public Builder setTrackerMaxOverlap(float trackerMaxOverlap) {
            this.trackerMaxOverlap = trackerMaxOverlap;
            return this;
        }

        public Builder setTrackerMinSize(float trackerMinSize) {
            this.trackerMinSize = trackerMinSize;
            return this;
        }

        public Builder setTrackerMarginalCorrelation(float trackerMarginalCorrelation) {
            this.trackerMarginalCorrelation = trackerMarginalCorrelation;
            return this;
        }

        public Builder setTrackerMinCorrelation(float trackerMinCorrelation) {
            this.trackerMinCorrelation = trackerMinCorrelation;
            return this;
        }

        public TfodParameters build() {
            return new TfodParameters(this);
        }
    }
}
