package org.darbots.darbotsftclib.libcore.calculations.algebraic_calculation;

public class OrderedValueSolver {
    public static final double INFINATELY_SMALL = 0.00000001;
    public static final double RESULT_NOSOLUTION = -1000000000000000.0;
    public static double solve(OrderedValueProvider valueProvider, double errorMargin, double independentVarMin, double independentVarMax, double desiredValue){
        if((valueProvider.orderIncremental() && independentVarMax > independentVarMin) || ((!valueProvider.orderIncremental()) && independentVarMax < independentVarMin)){
            return __solve(valueProvider,errorMargin,independentVarMin,independentVarMax,desiredValue);
        }else{
            return __solve(valueProvider,errorMargin,independentVarMax,independentVarMin,desiredValue);
        }
    }
    protected static double __solve(OrderedValueProvider valueProvider, double errorMargin, double independentVarMin, double independentVarMax, double desiredValue){
        double deltaMaxMin = independentVarMax - independentVarMin;
        if(Math.abs(deltaMaxMin) <= INFINATELY_SMALL){
            return RESULT_NOSOLUTION;
        }
        double middleVar = (independentVarMax + independentVarMin) / 2.0;
        double middleVal = valueProvider.valueAt(middleVar);
        if(Math.abs(middleVal - desiredValue) <= errorMargin){
            return middleVar;
        }else if(desiredValue < middleVal){
            //middleVar as new independentVarMax
            return __solve(valueProvider,errorMargin,independentVarMin,middleVar,desiredValue);
        }else{ //desiredValue > middleVal
            //middleVar as new independentVarMin
            return __solve(valueProvider,errorMargin,middleVar,independentVarMax,desiredValue);
        }
    }
}
