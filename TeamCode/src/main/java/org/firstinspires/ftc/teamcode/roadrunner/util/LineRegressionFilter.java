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
        double avg =0, avgI = (dataHist.size()+1)/2.0, num=0,denom=0;
        for(int i=0;i<dataHist.size();i++){
            avg += dataHist.get(i);
        }
        avg/= dataHist.size();
        for(int i=1;i<dataHist.size()+1;i++){
            num += (i-avgI)*(dataHist.get(i-1)-avg);
            denom += (i-avgI)*(i-avgI);
        }
        double coeff1 = num/denom, coeff0 = avg - coeff1*avgI;

        return coeff0+(dataHist.size())*coeff1;
    }
    public double regressedDist(double raw){
        if(dataHist.size()>=listSize){
            dataHist.remove(0);
        }
        dataHist.add(raw);
        return predictedValue()*(1-rawWeight)+raw*rawWeight;
    }
}
