/* Copyright © 2023 North Paulding High School Robotics Team 16757 */

package org.firstinspires.ftc.robotcontroller.internal;

public class NueronLayer {
    double[][] layer;
    double[][] layer_activated;
    double[][] weights;
    double[][] bias;
    double[][] input;
    double[][] weights_delta;
    double[][] bias_delta;
    double train_count;

    public NueronLayer(int inputLength, int length) {
        layer = new double[1][length];
        weights = new double[inputLength][length];
        bias = new double[1][length];
        for(int i = 0; i < bias[0].length; i++){
            bias[0][i] = 0.0;
        }
        weights_delta = new double[inputLength][length];
        bias_delta = new double[1][length];
        train_count = 0;
    }

    void LoadWeights(double[][] loadedWeights, double[][] loadedBias){
        weights = loadedWeights;
        bias = loadedBias;
    }

    //Not correctly implemented use evolution training method.
    public double[][] back_propagate(double[][] next_derivative){
        double[][] d1 = Matrix.tanhDerivative(layer);
        double[][] d2 = Matrix.elementMultiply(next_derivative, d1);
        double[][] d3 = Matrix.multiply(Matrix.transpose(input), d2);
        //weights_delta = Matrix.add(weights_delta, d3);
        //bias_delta = Matrix.add(bias_delta, d2);
        weights_delta = d3;
        bias_delta = d2;
        train_count += 1;
        return Matrix.multiply(d2, Matrix.transpose(weights));
    }
}
