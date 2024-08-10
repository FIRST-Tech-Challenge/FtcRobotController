/*
 * Copyright 2018 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.firstinspires.ftc.robotcore.external.tfod;

import java.util.List;

/**
 * Interface for TensorFlow Object Detector.
 *
 * @author Vasu Agrawal
 * @author lizlooney@google.com (Liz Looney)
 */
public interface TFObjectDetector {
    /**
     * Activates this TFObjectDetector so it starts recognizing objects.
     */
    void activate();

    /**
     * Deactivates this TFObjectDetector so it stops recognizing objects.
     */
    void deactivate();

    /**
     * Set the minimum confidence at which to keep recognitions.
     */
    public void setMinResultConfidence(float minResultConfidence);

    /**
     * Sets the number of pixels to obscure on the left, top, right, and bottom edges of each image
     * passed to the TensorFlow object detector. The size of the images are not changed, but the
     * pixels in the margins are colored black.
     */
    void setClippingMargins(int left, int top, int right, int bottom);

    /**
     * Indicates that only the zoomed center area of each image will be passed to the TensorFlow
     * object detector. For no zooming, set magnification to 1.0.
     */
    void setZoom(double magnification);

    /**
     * Returns the list of recognitions, but only if they are different than the last call to {@link #getUpdatedRecognitions()}.
     */
    List<Recognition> getUpdatedRecognitions();

    /**
     * Returns the list of recognitions.
     */
    List<Recognition> getRecognitions();

    /**
     * Perform whatever cleanup is necessary to release all acquired resources.
     */
    void shutdown();
}
