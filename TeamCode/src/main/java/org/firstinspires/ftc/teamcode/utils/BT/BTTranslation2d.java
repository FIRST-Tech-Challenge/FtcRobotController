package org.firstinspires.ftc.teamcode.utils.BT;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class BTTranslation2d extends Translation2d {
    public BTTranslation2d(double x, double y){
        super(x,y);
    }
    public Rotation2d getRotation(){
        return new Rotation2d(Math.atan2(getX(), getY()));
    }
}

