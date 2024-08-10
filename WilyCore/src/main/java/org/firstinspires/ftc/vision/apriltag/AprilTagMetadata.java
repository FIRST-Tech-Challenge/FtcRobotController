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

public class AprilTagMetadata
{
    public final int id;
    public final double tagsize;
    public final String name;
    public final DistanceUnit distanceUnit;
    public final VectorF fieldPosition;
    public final Quaternion fieldOrientation;

    /**
     * Add a tag to this tag library
     * @param id the ID of the tag
     * @param name a text name for the tag
     * @param tagsize the physical size of the tag in the real world (measured black edge to black edge)
     * @param fieldPosition a vector describing the tag's 3d translation on the field
     * @param distanceUnit the units used for size and fieldPosition
     * @param fieldOrientation a quaternion describing the tag's orientation on the field
     */
    public AprilTagMetadata(int id, String name, double tagsize, VectorF fieldPosition, DistanceUnit distanceUnit, Quaternion fieldOrientation)
    {
        this.id = id;
        this.name = name;
        this.tagsize = tagsize;
        this.fieldOrientation = fieldOrientation;
        this.fieldPosition = fieldPosition;
        this.distanceUnit = distanceUnit;
    }

    /**
     * Add a tag to this tag library
     * @param id the ID of the tag
     * @param name a text name for the tag
     * @param tagsize the physical size of the tag in the real world (measured black edge to black edge)
     * @param distanceUnit the units used for size and fieldPosition
     */
    public AprilTagMetadata(int id, String name, double tagsize, DistanceUnit distanceUnit)
    {
        this.id = id;
        this.name = name;
        this.tagsize = tagsize;
        this.fieldOrientation = Quaternion.identityQuaternion();
        this.fieldPosition = new VectorF(0,0,0);
        this.distanceUnit = distanceUnit;
    }
}
