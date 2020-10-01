package org.firstinspires.ftc.teamcode.RobotUtilities;

import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.HelperClasses.FloatPoint;

public class PiecewiseFunction {
    public ArrayList<FloatPoint> m_points = new ArrayList<>();

    public PiecewiseFunction(String visualString){
        //go through the string from bottom left to top right (kinda weird) assuming it's 20 by 9 chars

        int iter = visualString.length()-20;//this will be the bottom left
        while(true){

            if(visualString.charAt(iter) == '1'){
                int row = iter/20;//since lines are in groups of 20 chars, devide i by 20 and truncate decimals (int)
                int col = iter - (row*20);

                FloatPoint c = new FloatPoint((double) col/19.0,1.0-((double) row/8.0));
                m_points.add(c);
            }

            if(iter % 20 == 19){
                iter -= 40;
            }
            iter ++;

            if(iter < 0){break;}
        }
    }
    //if you want to manually set the points
    public PiecewiseFunction(ArrayList<FloatPoint> points){
        m_points = points;
    }

    //to use this function
    public double getVal(double x){
        double x1 = 0;
        double y1 = 0;
        double x2 = 0;
        double y2 = 0;

        for(int i = 0; i < m_points.size()-1; i ++){
            if(x >= m_points.get(i).x && x < m_points.get(i+1).x){
                x1 = m_points.get(i).x;
                y1 = m_points.get(i).y;
                x2 = m_points.get(i+1).x;
                y2 = m_points.get(i+1).y;
            }
        }
        //slope is change in y over change in x
        double slope = x2-x1 != 0 ? (y2-y1)/(x2-x1) : 100000;//avoiding a divide by 0
        /*
        To find y intercept use point-slope:
        y - y1 = m(x - x1)
        y - y1 = slope(0-x1)
        y = slope(-x1) + y1
        */
        double yintercept = (slope * -x1) + y1;
        return (x * slope) + yintercept;//plug the x value into our equation
    }
}
