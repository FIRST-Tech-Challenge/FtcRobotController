/* Copyright © 2023 North Paulding High School Robotics Team 16757 */

package org.firstinspires.ftc.robotcontroller.internal;

public class Matrix {
    public static double[][] multiply(double[][] matrix1, double[][] matrix2){
        double[][] cell = new double[matrix1.length][matrix2[0].length];

        for(int i = 0; i < matrix1.length; i++){
            for(int j = 0; j < matrix2[0].length; j++){
                for(int k = 0; k < matrix1[0].length; k++){
                    cell[i][j] += matrix1[i][k] * matrix2[k][j];
                }
            }
        }
        return cell;
    }

    public static double[][] elementMultiply(double[][] matrix1, double[][] matrix2) {
        double[][] cell = new double[matrix1.length][matrix1[0].length];

        for(int i = 0; i < cell.length; i++){
            for(int j = 0; j < cell[0].length; j++){
                cell[i][j] = matrix1[i][j] * matrix2[i][j];
            }
        }
        return cell;
    }

    public static double[][] scalarMultiply(double[][] matrix, double num){
        for(int i = 0; i < matrix.length; i++){
            for(int j = 0; j < matrix[0].length; j++){
                matrix[i][j] *= num;
            }
        }
        return matrix;
    }

    public static double[][] add(double[][] matrix1, double[][] matrix2){
        double[][] cell = new double[matrix1.length][matrix2[0].length];

        for(int i = 0; i < matrix1.length; i++) {
            for (int j = 0; j < matrix2[0].length; j++) {
                cell[i][j] = matrix1[i][j] + matrix2[i][j];
            }
        }
        return cell;
    }

    public static double[][] subtract(double[][] matrix1, double[][] matrix2){
        double[][] cell = new double[matrix1.length][matrix2[0].length];

        for(int i = 0; i < matrix1.length; i++) {
            for (int j = 0; j < matrix2[0].length; j++) {
                cell[i][j] = matrix1[i][j] - matrix2[i][j];
            }
        }
        return cell;
    }

    public static double[][] transpose(double[][] matrix){
        double[][] cell = new double[matrix[0].length][matrix.length];

        for(int i = 0; i < matrix.length; i++){
            for(int j = 0; j < matrix[0].length; j++){
                cell[j][i] = matrix[i][j];
            }
        }
        return cell;
    }

    public static double[][] solidMatrix(double num, int x, int y){
        double[][] cell = new double[x][y];

        for(int i = 0; i < cell.length; i++){
            for(int j = 0; j < cell[0].length; j++){
                cell[i][j] = num;
            }
        }
        return cell;
    }

    public static double[][] ReLU(double[][] matrix){
        double[][] cell = new double[matrix.length][matrix[0].length];

        for(int i = 0; i < matrix.length; i++){
            for(int j = 0; j < matrix[0].length; j++){
                if(matrix[i][j] > 0)
                    cell[i][j] = matrix[i][j];
                else
                    cell[i][j] = 0;
            }
        }
        return cell;
    }

    public static double[][] ReLUDerivative(double[][] matrix){
        double[][] cell = new double[matrix.length][matrix[0].length];

        for(int i = 0; i < matrix.length; i++){
            for(int j = 0; j < matrix[0].length; j++){
                if(matrix[i][j] > 0)
                    cell[i][j] = 1;
                else
                    cell[i][j] = 0;
            }
        }
        return cell;
    }

    public static double[][] tanh(double[][] matrix){
        double[][] cell = new double[matrix.length][matrix[0].length];

        for(int i = 0; i < matrix.length; i++){
            for(int j = 0; j < matrix[0].length; j++){
                cell[i][j] = (Math.exp(matrix[i][j]) - Math.exp(-matrix[i][j])) / (Math.exp(matrix[i][j]) + Math.exp(-matrix[i][j]));
            }
        }
        return cell;
    }

    public static double[][] tanhDerivative(double[][] matrix){
        double[][] cell = subtract(solidMatrix(1, matrix.length, matrix[0].length), elementMultiply(matrix, matrix));
        return cell;
    }

    public static double[][] error(double[][] output, double[][] correct){
        double[][] cell = scalarMultiply(elementMultiply(subtract(output, correct), subtract(output, correct)), 0.5);
        return cell;
    }

    public static double cost(double[][] output, double[][] correct){
        double cell = 0;

        for(int i = 0; i < output.length; i++){
            cell += Math.pow(correct[i][0] - output[i][0], 2);
        }

        return cell;
    }

    public static double[][] errorDerivative(double[][] output, double[][] correct){
        double[][] cell = subtract(output, correct);
        return cell;
    }
}
