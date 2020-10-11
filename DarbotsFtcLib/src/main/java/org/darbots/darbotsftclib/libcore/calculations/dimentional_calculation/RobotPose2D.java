/*
MIT License

Copyright (c) 2018 DarBots Collaborators

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
package org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation;


import java.io.Serializable;

public class RobotPose2D extends RobotVector2D implements Serializable {
    private static final long serialVersionUID = 1L;

    public RobotPose2D(double X, double Y, double ZRotation) {
        super(X, Y, XYPlaneCalculations.normalizeDeg(ZRotation));
    }

    public RobotPose2D(RobotPoint2D Point, double ZRotation) {
        super(Point, XYPlaneCalculations.normalizeDeg(ZRotation));
    }

    public RobotPose2D(RobotVector2D Pos2D) {
        super(Pos2D.X,Pos2D.Y,XYPlaneCalculations.normalizeDeg(Pos2D.m_RotationZ));
    }

    public RobotPose2D(RobotPose2D Pose2D){
        super(Pose2D);
        this.m_RotationZ = Pose2D.m_RotationZ;
    }

    public void setValues(double X, double Y, double RotationZ){
        this.X = X;
        this.Y = Y;
        this.m_RotationZ = XYPlaneCalculations.normalizeDeg(RotationZ);
    }
    public void setValues(RobotVector2D pose2D){
        this.X = pose2D.X;
        this.Y = pose2D.Y;
        this.m_RotationZ = XYPlaneCalculations.normalizeDeg(pose2D.m_RotationZ);
    }
    public void offsetValues(double X, double Y, double RotationZ){
        this.X += X;
        this.Y += Y;
        this.m_RotationZ = XYPlaneCalculations.normalizeDeg(this.m_RotationZ + RotationZ);
    }
    public void setRotationZ(double RotationZ){
        this.m_RotationZ = XYPlaneCalculations.normalizeDeg(RotationZ);
    }

    public boolean equals(RobotPose2D Pose){
        if(this.getRotationZ() == Pose.getRotationZ() && this.X == Pose.X && this.Y == Pose.Y){
            return true;
        }else{
            return false;
        }
    }
}
