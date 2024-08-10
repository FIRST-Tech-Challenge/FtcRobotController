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

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;

/**
 * A tag library contains metadata about tags such as
 * their ID, name, size, and 6DOF position on the field
 */
public class AprilTagLibrary
{
    private final AprilTagMetadata[] data;

    private AprilTagLibrary(AprilTagMetadata[] data)
    {
        this.data = data;
    }

    /**
     * Get the metadata of all tags in this library
     * @return the metadata of all tags in this library
     */
    public AprilTagMetadata[] getAllTags()
    {
        return data;
    }

    /**
     * Get the metadata for a specific tag in this library
     * @param id the ID of the tag in question
     * @return either {@link AprilTagMetadata} for the tag, or
     * NULL if it isn't in this library
     */
    public AprilTagMetadata lookupTag(int id)
    {
        for (AprilTagMetadata tagMetadata : data)
        {
            if (tagMetadata.id == id)
            {
                return tagMetadata;
            }
        }

        return null;
    }

    public static class Builder
    {
        private ArrayList<AprilTagMetadata> data = new ArrayList<>();
        private boolean allowOverwrite = false;

        /**
         * Set whether to allow overwriting an existing entry in the tag
         * library with a new entry of the same ID
         * @param allowOverwrite whether to allow overwrite
         * @return the {@link Builder} object, to allow for method chaining
         */
        public Builder setAllowOverwrite(boolean allowOverwrite)
        {
            this.allowOverwrite = allowOverwrite;
            return this;
        }

        /**
         * Add a tag to this tag library
         * @param aprilTagMetadata the tag to add
         * @return the {@link Builder} object, to allow for method chaining
         * @throws RuntimeException if trying to add a tag that already exists
         * in this library, unless you called {@link #setAllowOverwrite(boolean)}
         */
        public Builder addTag(AprilTagMetadata aprilTagMetadata)
        {
            for (AprilTagMetadata m : data)
            {
                if (m.id == aprilTagMetadata.id)
                {
                    if (allowOverwrite)
                    {
                        // This is ONLY safe bc we immediately stop iteration here
                        data.remove(m);
                        break;
                    }
                    else
                    {
                        throw new RuntimeException("You attempted to add a tag to the library when it already contains a tag with that ID. You can call .setAllowOverwrite(true) to allow overwriting the existing entry");
                    }
                }
            }

            data.add(aprilTagMetadata);
            return this;
        }

        /**
         * Add a tag to this tag library
         * @param id the ID of the tag
         * @param name a text name for the tag
         * @param size the physical size of the tag in the real world (measured black edge to black edge)
         * @param fieldPosition a vector describing the tag's 3d translation on the field
         * @param distanceUnit the units used for size and fieldPosition
         * @param fieldOrientation a quaternion describing the tag's orientation on the field
         * @return the {@link Builder} object, to allow for method chaining
         * @throws RuntimeException if trying to add a tag that already exists
         * in this library, unless you called {@link #setAllowOverwrite(boolean)}
         */
        public Builder addTag(int id, String name, double size, VectorF fieldPosition, DistanceUnit distanceUnit, Quaternion fieldOrientation)
        {
            return addTag(new AprilTagMetadata(id, name, size, fieldPosition, distanceUnit, fieldOrientation));
        }

        /**
         * Add a tag to this tag library
         * @param id the ID of the tag
         * @param name a text name for the tag
         * @param size the physical size of the tag in the real world (measured black edge to black edge)
         * @param distanceUnit the units used for size and fieldPosition
         * @return the {@link Builder} object, to allow for method chaining
         * @throws RuntimeException if trying to add a tag that already exists
         * in this library, unless you called {@link #setAllowOverwrite(boolean)}
         */
        public Builder addTag(int id, String name, double size, DistanceUnit distanceUnit)
        {
            return addTag(new AprilTagMetadata(id, name, size, new VectorF(0,0,0), distanceUnit, Quaternion.identityQuaternion()));
        }

        /**
         * Add multiple tags to this tag library
         * @param library an existing tag library to add to this one
         * @return the {@link Builder} object, to allow for method chaining
         * @throws RuntimeException if trying to add a tag that already exists
         * in this library, unless you called {@link #setAllowOverwrite(boolean)}
         */
        public Builder addTags(AprilTagLibrary library)
        {
            for (AprilTagMetadata m : library.getAllTags())
            {
                // Delegate to this implementation so we get duplicate checking for free
                addTag(m);
            }
            return this;
        }

        /**
         * Create an {@link AprilTagLibrary} object from the specified tags
         * @return an {@link AprilTagLibrary} object
         */
        public AprilTagLibrary build()
        {
            return new AprilTagLibrary(data.toArray(new AprilTagMetadata[0]));
        }
    }
}
