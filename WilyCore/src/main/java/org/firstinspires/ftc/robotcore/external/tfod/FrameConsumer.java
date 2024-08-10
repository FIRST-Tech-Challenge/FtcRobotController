/*
 * Copyright (C) 2022 Google LLC
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
 * FrameConsumer is an interface that is implemented by the FTC TensorFlow object detector engine
 * in order to consume frames from a FrameGenerator.
 */
public interface FrameConsumer {
// ###    /**
// ###     * init should be called from a FrameGenerator once, immediately after the bitmap has been
// ###     * created. When called, the FTC TensorFlow object detector engine will initialize the internal
// ###     * pipeline that will process frames.
// ###     */
// ###    void init(Bitmap input);
// ###
// ###    /**
// ###     * processFrame should be called from a FrameGenerator each time the bitmap (previously passed to
// ###     * init) contains new data from the camera. It is expected that processFrame be called from the
// ###     * same thread that previously called init.
// ###     */
// ###    CanvasAnnotator processFrame();
}
