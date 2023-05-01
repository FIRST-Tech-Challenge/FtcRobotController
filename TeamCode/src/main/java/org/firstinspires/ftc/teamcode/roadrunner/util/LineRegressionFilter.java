package org.firstinspires.ftc.teamcode.roadrunner.util;

import java.util.ArrayList;

public class LineRegressionFilter {
    ArrayList<Double> dataHist;
    double rawWeight;
    int listSize;
    public LineRegressionFilter(double trust, int listLength){
        dataHist = new ArrayList<>();
        rawWeight = trust;
        listSize = listLength;
    }
    public double predictedValue(){
        double regVal = 0;
        return regVal;
    }
    public double regressedDist(double raw){
        if(dataHist.size()>=listSize){
            dataHist.remove(0);
        }
        dataHist.add(raw);
        return predictedValue()*(1-rawWeight)+raw*rawWeight;
    }
}
