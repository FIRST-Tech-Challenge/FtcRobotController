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


/**
 * FrameGenerator provides a unified interface for receiving consecutive frames from a sequence.
 *
 * <p>Typically, consecutive frames will be different (as if generated from a video, camera, or
 * other time-dependent sequence), but this is not required by the interface.
 *
 * @author Vasu Agrawal
 * @author lizlooney@google.com (Liz Looney)
 */
public interface FrameGenerator {

    /**
     * Returns the CameraInformation for this FrameGenerator.
     */
    CameraInformation getCameraInformation();

    /**
     * Attach the given FrameConsumer to this FrameGenerator. If frameConsumer is null, detach the
     * previous attached FrameConsumer.
     * The FrameConsumer will accept frames and send them to the TensorFlow object detector engine
     * and an object tracker (if enabled).
     */
    void setFrameConsumer(FrameConsumer frameConsumer);
}
