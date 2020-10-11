package org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation;

public class DarbotsDerivative {
    public static final double DERIVATIVE_VERTICLE_TANGENT_YPOSITIVE = 100000000;
    public static final double DERIVATIVE_VERTICLE_TANGENT_YNEGATIVE = -100000000;
    public static final double DERIVATIVE_VERTICLE_TANGENT_YZERO = -500000000;
    public double deltaX = 0;
    public double deltaY = 0;
    public DarbotsDerivative(double deltaX, double deltaY){
        this.deltaX = deltaX;
        this.deltaY = deltaY;
    }
    public DarbotsDerivative(DarbotsDerivative oldDerivative){
        this.deltaX = oldDerivative.deltaX;
        this.deltaY = oldDerivative.deltaY;
    }
    public double computeDyOverDx(){
        double deltaX, deltaY = 0;
        if(this.deltaX < 0){
            deltaX = -this.deltaX;
            deltaY = -this.deltaY;
        }else{
            deltaX = this.deltaX;
            deltaY = this.deltaY;
        }
        if(deltaX == 0){
            if(deltaY > 0){
                return DERIVATIVE_VERTICLE_TANGENT_YPOSITIVE;
            }else if(deltaY == 0){
                return DERIVATIVE_VERTICLE_TANGENT_YZERO;
            }else{ //deltaY < 0
                return DERIVATIVE_VERTICLE_TANGENT_YNEGATIVE;
            }
        }else{
            return deltaY / deltaX;
        }
    }
}
