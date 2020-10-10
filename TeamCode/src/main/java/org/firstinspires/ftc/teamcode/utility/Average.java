package org.firstinspires.ftc.teamcode.utility;


import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * For doing weighted and unweighted sums
 * This primarily handles edge cases
 */
public class Average {
    double sum = 0;
    double weightSum = 0;

    public void add(double value, double weight){
        if (Double.isFinite(value)) {
            sum += value * weight;
            weightSum += weight;
        }
    }

    public void add(double value){
        add(value, 1);
    }

    public double average(){
        if (weightSum == 0) return 0;
        return sum / weightSum;
    }

    public <T> double ofAll(List<T> list, Function<T, Double> addend){
        reset();
        for(T element : list){
            add(addend.apply(element));
        }
        return average();
    }
    public <T> double ofAll(List<T> list, Consumer<T> addend){
        reset();
        for(T element : list){
            addend.accept(element);
        }
        return average();
    }


    public void reset(){
        sum = 0;
        weightSum = 0;
    }
}
