package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.hardware.rev.RevColorSensorV3;


public class ColorSensor {

    RevColorSensorV3 REVColor;
    public ColorSensor(RevColorSensorV3 _REVSensor){
        REVColor = _REVSensor;
    }

    //senzorul verfica daca culoare pe care o vede se afla in intervalul RGB pentru galben +/-
    public boolean checkForYellow(){
        if(REVColor.red() <= 255 && REVColor.red() >= 128 && REVColor.green() <= 255 && REVColor.green() >= 128 && REVColor.blue() == 0){
            return true;
        }
        else return false;

    }

    //senzorul verfica daca culoare pe care o vede se afla in intervalul RGB pentru alb +/-
    public boolean checkForWhite(){
        if(REVColor.red() <= 50 && REVColor.red() >= 0 && REVColor.green() <=
                50 && REVColor.green() >= 0 && REVColor.blue() <= 50 && REVColor.blue() >= 0){
            return true;
        }
        else return false;
    }
}
