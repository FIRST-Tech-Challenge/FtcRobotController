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

public class RobotPose3D implements Serializable {
    private static final long serialVersionUID = 1L;
    public double X;
    public double Y;
    public double Z;
    protected double m_RotationX;
    protected double m_RotationZ;
    protected double m_RotationY;
    public RobotPose3D(double X, double Y, double Z, double RotationX, double RotationY, double RotationZ){
        this.X = X;
        this.Y = Y;
        this.Z = Z;
        this.m_RotationX = XYPlaneCalculations.normalizeDeg(RotationX);
        this.m_RotationY = XYPlaneCalculations.normalizeDeg(RotationY);
        this.m_RotationZ = XYPlaneCalculations.normalizeDeg(RotationZ);
    }
    public RobotPose3D(RobotPose2D Pos2D, double Z, double RotationX, double RotationY){
        this.X = Pos2D.X;
        this.Y = Pos2D.Y;
        this.Z = Z;
        this.m_RotationX = XYPlaneCalculations.normalizeDeg(RotationX);
        this.m_RotationZ = Pos2D.getRotationZ();
        this.m_RotationY = XYPlaneCalculations.normalizeDeg(RotationY);
    }
    public RobotPose3D(RobotPose3D Pos3D){
        this.X = Pos3D.X;
        this.Y = Pos3D.Y;
        this.Z = Pos3D.Z;
        this.m_RotationX = Pos3D.m_RotationX;
        this.m_RotationY = Pos3D.m_RotationY;
        this.m_RotationZ = Pos3D.m_RotationZ;
    }
    public double getDistanceToOrigin(){
        return (Math.sqrt(Math.pow(this.X,2) + Math.pow(this.Y,2) + Math.pow(this.Z,2)));
    }
    public RobotPose2D get2DPosition(){
        return new RobotPose2D(this.X,this.Y,this.getRotationZ());
    }
    public double getRotationX(){
        return this.m_RotationX;
    }
    public void setRotationX(double RotationX){
        this.m_RotationX = XYPlaneCalculations.normalizeDeg(RotationX);
    }
    public double getRotationY(){
        return this.m_RotationY;
    }
    public void setRotationY(double RotationY){
        this.m_RotationY = XYPlaneCalculations.normalizeDeg(RotationY);
    }
    public double getRotationZ(){
        return this.m_RotationZ;
    }
    public void setRotationZ(double RotationZ){
        this.m_RotationZ = XYPlaneCalculations.normalizeDeg(RotationZ);
    }
}
