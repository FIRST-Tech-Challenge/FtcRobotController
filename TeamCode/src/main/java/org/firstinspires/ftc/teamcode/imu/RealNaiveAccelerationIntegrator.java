/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.imu;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.meanIntegrate;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;




import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class RealNaiveAccelerationIntegrator implements BNO055IMU.AccelerationIntegrator
{
    //------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------

    BNO055IMU.Parameters parameters;
    Position position;
    Velocity velocity;
    Acceleration acceleration;

    public Position getPosition() { return this.position; }
    public Velocity getVelocity() { return this.velocity; }
    public Acceleration getAcceleration() { return this.acceleration; }

    //------------------------------------------------------------------------------------------
    // Construction
    //------------------------------------------------------------------------------------------

    RealNaiveAccelerationIntegrator()
    {
        this.parameters = null;
        this.position = new Position();
        this.velocity = new Velocity();
        this.acceleration = null;
    }

    //------------------------------------------------------------------------------------------
    // Operations
    //------------------------------------------------------------------------------------------

    @Override public void initialize(@NonNull BNO055IMU.Parameters parameters,  Position initialPosition, Velocity initialVelocity)
    {
        this.parameters = parameters;
        this.position = initialPosition != null ? initialPosition : this.position;
        this.velocity = initialVelocity != null ? initialVelocity : this.velocity;
        this.acceleration = null;
    }

    @Override public void update(Acceleration linearAcceleration)
    {
        // We should always be given a timestamp here
        if (linearAcceleration.acquisitionTime != 0)
        {
            // We can only integrate if we have a previous acceleration to baseline from
            if (acceleration != null)
            {
                Acceleration accelPrev    = acceleration;
                Velocity     velocityPrev = velocity;

                acceleration = linearAcceleration;

                if (accelPrev.acquisitionTime != 0)
                {
                    Velocity deltaVelocity = meanIntegrate(acceleration, accelPrev);
                    velocity = plus(velocity, deltaVelocity);
                }

                if (velocityPrev.acquisitionTime != 0)
                {
                    Position deltaPosition = meanIntegrate(velocity, velocityPrev);
                    position = plus(position, deltaPosition);
                }

                if (parameters != null && parameters.loggingEnabled)
                {
                    RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", (acceleration.acquisitionTime - accelPrev.acquisitionTime)*1e-9, acceleration, velocity, position);
                }
            }
            else
                acceleration = linearAcceleration;
        }
    }
}
