package org.firstinspires.ftc.teamcode.roadrunner.util;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import java.util.ArrayList;

public class LineRegressionFilter {
    ArrayList<Double> dataHist;
    double rawWeight;
    int listSize;
    double blop=5;
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
        packet.put("num", num);
        packet.put("denom", denom);
        double coeff1 = num/denom, coeff0 = avg - coeff1*avgI;
        if(coeff1!=0) {
            blop = coeff1;
        }
        packet.put("tempSlop", blop);
        return coeff0+(dataHist.size())*coeff1;
    }
    public double regressedDist(double raw){
        if(dataHist.size()>=listSize){
            packet.put("data", dataHist.toString());
            dataHist.remove(0);
        }
        dataHist.add(raw);
        return predictedValue()*(1-rawWeight)+raw*rawWeight;
    }
    public double getSlope() {
        return blop;
    }
}
