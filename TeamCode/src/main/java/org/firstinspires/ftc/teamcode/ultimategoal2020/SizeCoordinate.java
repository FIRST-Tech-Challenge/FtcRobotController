package org.firstinspires.ftc.teamcode.ultimategoal2020;

import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;

import java.util.ArrayList;

public class SizeCoordinate {
    CsysDirection csysDirection;
    double sizeValue;

    public SizeCoordinate(CsysDirection dir, double value){
        this.csysDirection = dir;
        this.sizeValue = value;
    }

    public CsysDirection getCsysDirection() {
        return csysDirection;
    }

    public double getSizeValue() {
        return sizeValue;
    }

    public static double getSizeFromCoordinates(CsysDirection dir, ArrayList<SizeCoordinate> sizeCoordinates){
        //Loop through an array of coordinates to return size value
        double sizeValue = 0;

        //Quick error check on the array
        if(sizeCoordinates == null) sizeCoordinates = new ArrayList<>(sizeCoordinates);  //create empty array if null

        for(SizeCoordinate sc: sizeCoordinates) {
            if (sc.csysDirection == dir) {
                sizeValue = sc.sizeValue;
                break;
            }
        }

        return sizeValue;
    }
}
