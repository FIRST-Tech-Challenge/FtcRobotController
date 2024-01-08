/* Copyright © 2023 North Paulding High School Robotics Team 16757 */

package org.firstinspires.ftc.robotcontroller.internal;

import android.os.Environment;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;
import java.util.Scanner;

public class WolfNet {
    double[][] input;
    public NueronLayer output;
    NueronLayer[] hidden_layers;
    String weight_file;
    Scanner weightS;

    public static double learningRate;

    public WolfNet(int input_size, int output_size, int hidden_size, String weightF, double learnR){
        input = new double[1][input_size];
        hidden_layers = new NueronLayer[hidden_size];
        for (int i = 0; i < hidden_layers.length; i++){
            int lastInLength;
            if(i == 0)
                lastInLength = input_size;
            else
                lastInLength = hidden_layers[i - 1].layer[0].length;
            //hidden_layers[i] = new NueronLayer(lastInLength, (lastInLength - ((input_size - output_size) / hidden_size)));
            hidden_layers[i] = new NueronLayer(lastInLength, 2);
        }
        output = new NueronLayer(hidden_layers[hidden_layers.length - 1].layer[0].length, output_size);
        weight_file = weightF;
        try{
            weightS = new Scanner(new File(Environment.getExternalStorageDirectory().getPath() + "/" + weightF + ".txt"));
        }catch(IOException e){

        }
        learningRate = learnR;
    }
    public WolfNet(WolfNet network, String weightF){
        input = network.input;
        hidden_layers = network.hidden_layers;
        output = network.output;
        weight_file = weightF;
        try{
            weightS = new Scanner(new File(Environment.getExternalStorageDirectory().getPath() + "/" + weightF + ".txt"));
        }catch(IOException e){

        }
        learningRate = network.learningRate;
        MutateWeights();
    }

    void LoadWeights(){
        double[][] weights;
        double[][] bias;

        for(int i = 0; i < hidden_layers.length; i++){
            weights = new double[hidden_layers[i].weights.length][hidden_layers[i].weights[0].length];
            bias = new double[1][hidden_layers[i].bias[0].length];
            for(int j = 0; j < weights.length; j++) {
                for(int k = 0; k < weights[0].length; k++) {
                    if(weightS.hasNextLine()){
                        weights[j][k] = Double.parseDouble(weightS.nextLine());
                    }
                }
            }
            for(int j = 0; j < bias[0].length; j ++){
                if(weightS.hasNextLine()){
                    bias[0][j] = Double.parseDouble(weightS.nextLine());
                }
            }
            hidden_layers[i].LoadWeights(weights, bias);
        }
        weights = new double[output.weights.length][output.weights[0].length];
        bias = new double[1][output.bias[0].length];
        for(int i = 0; i < weights.length; i++){
            for(int j = 0; j < weights[0].length; j++){
                if(weightS.hasNextLine()){
                    weights[i][j] = Double.parseDouble(weightS.nextLine());
                }
            }
            for(int j = 0; j < bias[0].length; j ++){
                if(weightS.hasNextLine()){
                    bias[0][j] = Double.parseDouble(weightS.nextLine());
                }
            }
        }
        output.LoadWeights(weights, bias);
    }

    void SaveWeights(){
        FileWriter file_writer;
        try{
            new FileWriter(Environment.getExternalStorageDirectory().getPath() + "/" + weight_file + ".txt", false).close();
            file_writer = new FileWriter(Environment.getExternalStorageDirectory().getPath() + "/" + weight_file + ".txt", true);
            for (int i = 0; i < hidden_layers.length; i++) {
                for (int j = 0; j < hidden_layers[i].weights.length; j++) {
                    for (int k = 0; k < hidden_layers[i].weights[0].length; k++) {
                        try{
                            file_writer.write(Double.toString(hidden_layers[i].weights[j][k]) + "\n");
                        }catch(IOException e){

                        }
                    }
                }
                for (int j = 0; j < hidden_layers[i].bias[0].length; j++){
                    try{
                        file_writer.write(Double.toString(hidden_layers[i].bias[0][j]) + "\n");
                    }catch(IOException e){

                    }
                }
            }
            for(int i = 0; i < output.weights.length; i++){
                for(int j = 0; j < output.weights[0].length; j++){
                    try{
                        file_writer.write(Double.toString(output.weights[i][j]) + "\n");
                    }catch(IOException e){

                    }
                }
            }
            for (int i = 0; i < output.bias[0].length; i++){
                try{
                    file_writer.write(Double.toString(output.bias[0][i]) + "\n");
                }catch(IOException e){

                }
            }
            try{
                file_writer.close();
            }catch(IOException e){

            }
        }catch(IOException e){

        }
    }

    void ResetWeights(){
        Random random = new Random();
        for(int i = 0; i < hidden_layers.length; i++){
            for(int j = 0; j < hidden_layers[i].weights.length; j++){
                for(int k = 0; k < hidden_layers[i].weights[0].length; k++){
                    hidden_layers[i].weights[j][k] = (random.nextDouble() * 2) - 1;
                    //hidden_layers[i].weights[j][k] = random.nextDouble();
                }
            }
        }
        for(int i = 0; i < hidden_layers.length; i++){
            for(int j = 0; j < hidden_layers[i].bias[0].length; j++){
                hidden_layers[i].bias[0][j] = (random.nextDouble() * 2) - 1;
            }
        }
        for(int i = 0; i < output.weights.length; i++){
            for(int j = 0; j < output.weights[0].length; j++){
                output.weights[i][j] = (random.nextDouble() * 2) - 1;
                //output.weights[i][j] = random.nextDouble();
            }
        }
        for(int i = 0; i < output.bias[0].length; i++){
            output.bias[0][i] = (random.nextDouble() * 2) - 1;
        }
    }
    void MutateWeights(){
        Random random = new Random();
        int layer = random.nextInt(hidden_layers.length);
        int x = random.nextInt(hidden_layers[layer].weights.length);
        int y = random.nextInt(hidden_layers[layer].weights[0].length);
        hidden_layers[layer].weights[x][y] = (random.nextDouble() * 2) - 1;
    }

    void GetOutput(double[][] in){
        input = in;

        for(int i = 0; i < hidden_layers.length; i++){
            if(i == 0) {
                hidden_layers[0].layer = Matrix.multiply(input, hidden_layers[0].weights);
                hidden_layers[0].layer = Matrix.add(hidden_layers[0].layer, hidden_layers[0].bias);
                hidden_layers[0].layer = Matrix.tanh(hidden_layers[0].layer);
                hidden_layers[0].input = input;
            }else {
                hidden_layers[i].layer = Matrix.multiply(hidden_layers[i - 1].layer, hidden_layers[i].weights);
                hidden_layers[i].layer = Matrix.add(hidden_layers[i].layer, hidden_layers[i].bias);
                hidden_layers[i].layer = Matrix.tanh(hidden_layers[i].layer);
                hidden_layers[i].input = hidden_layers[i - 1].layer;
            }
        }
        output.layer = Matrix.multiply(hidden_layers[hidden_layers.length - 1].layer, output.weights);
        output.layer = Matrix.add(output.layer, output.bias);
        //output.layer = Matrix.tanh(output.layer);
        output.input = hidden_layers[hidden_layers.length - 1].layer_activated;
    }

    void Train(double[][] in, double[][] correct){
        GetOutput(in);
        double[][] derivative = Matrix.errorDerivative(output.layer, correct);
        derivative = output.back_propagate(derivative);
        for(int i = hidden_layers.length - 1; i >= 0; i--){
            derivative = hidden_layers[i].back_propagate(derivative);
        }

        UpdateWeights();
    }

    void UpdateWeights(){
        double[][] weights;
        double[][] bias;

        for(int i = 0; i < hidden_layers.length; i++){
            //weights = Matrix.scalarMultiply(hidden_layers[i].weights_delta, (1 / hidden_layers[i].train_count));
            //bias = Matrix.scalarMultiply(hidden_layers[i].bias_delta, (1 / hidden_layers[i].train_count));
            weights = hidden_layers[i].weights_delta;
            bias = hidden_layers[i].bias_delta;

            hidden_layers[i].weights = Matrix.subtract(hidden_layers[i].weights, Matrix.scalarMultiply(weights, learningRate));
            hidden_layers[i].bias = Matrix.subtract(hidden_layers[i].bias, Matrix.scalarMultiply(bias, learningRate));

            //hidden_layers[i].weights_delta = Matrix.solidMatrix(0, hidden_layers[i].weights_delta.length, hidden_layers[i].weights_delta[0].length);
            //hidden_layers[i].bias_delta = Matrix.solidMatrix(0, hidden_layers[i].bias_delta.length, hidden_layers[i].bias_delta[0].length);
            //hidden_layers[i].train_count = 0;
        }
        //weights = Matrix.scalarMultiply(output.weights_delta, (1 / output.train_count));
        //bias = Matrix.scalarMultiply(output.bias_delta, (1 / output.train_count));
        weights = output.weights_delta;
        bias = output.bias_delta;

        output.weights = Matrix.subtract(output.weights, Matrix.scalarMultiply(weights, learningRate));
        output.bias = Matrix.subtract(output.bias, Matrix.scalarMultiply(bias, learningRate));

        //output.weights_delta = Matrix.solidMatrix(0, output.weights_delta.length, output.weights_delta[0].length);
        //output.bias_delta = Matrix.solidMatrix(0, output.bias_delta.length, output.bias_delta[0].length);
        //output.train_count = 0;
    }
}