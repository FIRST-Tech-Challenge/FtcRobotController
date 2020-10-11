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

package org.darbots.darbotsftclib.libcore.sensors.motion_related;


import androidx.annotation.NonNull;

import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;

public class RobotMotion {
    protected RobotMotor m_Motor;
    protected RobotWheel m_Wheel;

    public RobotMotion(@NonNull RobotMotor MotorController, @NonNull RobotWheel RobotWheel){
        this.m_Motor = MotorController;
        this.m_Wheel = RobotWheel;
    }
    public RobotMotor getMotor(){
        return this.m_Motor;
    }
    public void setMotor(@NonNull RobotMotor Motor){
        this.m_Motor = Motor;
    }
    public RobotWheel getRobotWheel(){
        return this.m_Wheel;
    }
    public void setRobotWheel(RobotWheel Wheel){
        this.m_Wheel = Wheel;
    }

}
