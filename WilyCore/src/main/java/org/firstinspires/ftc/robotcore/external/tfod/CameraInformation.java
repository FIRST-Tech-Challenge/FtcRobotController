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

import java.util.Objects;

/**
 * Class representing information about the camera needed for object detection.
 *
 * @author lizlooney@google.com (Liz Looney)
 */
public class CameraInformation {
    /**
     * The width of frames from the camera, in pixels.
     */
    public final int width;
    /**
     * The height of frames from the camera, in pixels.
     */
    public final int height;
    /**
     * The rotation of the camera, in degrees. Must be 0, 90, 180, or 270.
     */
    public final int rotation;
    /**
     * The horizontal focal length of the camera, in pixels. The focal length is used by a
     * Recognition to estimate the angle to the detected object.
     */
    public final float horizontalFocalLength;
    /**
     * The vertical focal length of the camera, in pixels. The focal length is used by a
     * Recognition to estimate the angle to the detected object.
     */
    public final float verticalFocalLength;

    /**
     * Constructs a CameraInformation instance with the given parameters.
     */
    public CameraInformation(int width, int height, int rotation,
                             float horizontalFocalLength, float verticalFocalLength) {
        this.width = width;
        this.height = height;
        this.rotation = rotation;
        this.horizontalFocalLength = horizontalFocalLength;
        this.verticalFocalLength = verticalFocalLength;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        CameraInformation ci = (CameraInformation) o;
        return width == ci.width
                && height == ci.height
                && rotation == ci.rotation
                && horizontalFocalLength == ci.horizontalFocalLength
                && verticalFocalLength == ci.verticalFocalLength;
    }

    @Override
    public int hashCode() {
        return Objects.hash(width, height, rotation, horizontalFocalLength, verticalFocalLength);
    }
}
