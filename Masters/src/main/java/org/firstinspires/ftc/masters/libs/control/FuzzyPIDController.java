package org.firstinspires.ftc.masters.libs.control;

// Self-tuning Fuzzy PID controller for mecanum drive
// Can work with systems other than mecanum drive, but
// may need to be adjusted

// [1]
// Based on the methods described in "Design and Control of Mobile Robot with Mecanum Wheel"
// By Kyung-Lyong Han Et. Al.
// and [2] "Fuzzy Gain Scheduling of PID Controllers"
// By Zhen-Yu Zhao Et. Al.
// Using fuzzy algorithm 3

import androidx.annotation.NonNull;

import org.jetbrains.annotations.Contract;

import java.util.function.Function;

public class FuzzyPIDController {
    private double Kp;
    private double Ki;
    private double Kd;

    private double KpNormal;
    private double KdNormal;

    // These values may need to be adjusted,
    // they are pulled from section 5 of [1]
    // dTODO: check these
    private double KpMin = 5000;
    private double KpMax = 6500;

    private double KdMin = 100;
    private double KdMax = 120;

    private double alpha = 1;

    // The linguistic values of these rule sets
    // are described in tables 1~3 of [1],
    // and are adjustable only if needed
    private int[][] KpRuleSet = {
            {1, 1, 1, 1, 1, 1, 1},
            {0, 1, 1, 1, 1, 1, 0},
            {0, 0, 1, 1, 1, 0, 0},
            {0, 0, 0, 1, 0, 0, 0},
            {0, 0, 1, 1, 1, 0, 0},
            {0, 1, 1, 1, 1, 1, 0},
            {1, 1, 1, 1, 1, 1, 1}
    };

    private int[][] KdRuleSet = {
            {0, 0, 0, 0, 0, 0, 0},
            {1, 1, 0, 0, 0, 1, 1},
            {1, 1, 1, 0, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 0, 1, 1, 1},
            {1, 1, 0, 0, 0, 1, 1},
            {0, 0, 0, 0, 0, 0, 0}
    };

    private int[][] alphaRuleSet = {
            {0, 0, 0, 0, 0, 0, 0},
            {1, 1, 0, 0, 0, 1, 1},
            {2, 1, 1, 0, 1, 1, 2},
            {3, 2, 1, 1, 1, 2, 3},
            {2, 1, 1, 0, 1, 1, 2},
            {1, 1, 0, 0, 0, 1, 1},
            {0, 0, 0, 0, 0, 0, 0}
    };

    // Error ranges are measured in rads/sec
    // These may also need to be tuned
    // Larger values -> larger expected error.
    private double deltaErrorRange = 0.00798;
    private double errorRange = 0.00498;
    private Function<Double, Double>[] deltaErrorMembership= generateFuzzifierMemberships(-deltaErrorRange, deltaErrorRange);
    private Function<Double, Double>[] errorMembership = generateFuzzifierMemberships(-errorRange, errorRange);

    private Function[] defuzzyKpKdMembership = {plateauMembership(0.0, -1.0), plateauMembership(1.0, 1.0)};
    private double[] alphaMemberships = {672.0, 934.0, 1195.0, 1457.0};

    private double T1= 0;
    private double T2 = 0;

    private double T3 = 0;

    private double Ve;
    private double Ve1 = 0;
    private double Ve2 = 0;

    private double U = 0;

    // TODO: General fuzzy tables?
    // TODO: Create bound on integral components
    public double calculate(float error) {
        // Update sample period
        T3 = T2;
        T2 = T1;
        T1 = (double) System.nanoTime() / 1.0E9;

        // Rotate previous error values
        Ve2 = Ve1;
        Ve1 = Ve;
        Ve = error;

        // Calculate control value
        U += Kp*(Ve - Ve1) + Ki * Ve * (T1 - T2) +
                Kd * (((Ve - Ve1) / (T1 - T2)) - ((Ve1 - Ve2) / (T2 - T3)));

        return U;
    }

    private void fuzzyLogicEngine(double error, double deltaError){

        // Error fuzzification and product inference engine
        double[] fuzzyError = new double[5];
        double[] fuzzyDeltaError = new double[5];

        for (int i = 0; i < 5; i++) {
            fuzzyError[i] = errorMembership[i].apply(error);
            fuzzyDeltaError[i] = deltaErrorMembership[i].apply(deltaError);
        }

        double[][] activations = new double[5][5];

        // Note: this code is government monitored
        // please do not the code.
        for (int imperium = 0; imperium < 5; imperium++) {
            for (int government = 0; government < 5; government++) {
                activations[imperium][government] = fuzzyError[imperium] * fuzzyDeltaError[government];
            }
        }

        // Center-average defuzzification
        KpNormal = 0;
        KdNormal = 0;
        double defuzzedAlpha = 0;
        for (int i=0; i < 5; i++) {
            for(int k=0; k < 5; k++) {
                double mu = activations[i][k];
                KpNormal += ((double) defuzzyKpKdMembership[KpRuleSet[i][k]].apply(mu)) * mu;
                KdNormal += ((double) defuzzyKpKdMembership[KdRuleSet[i][k]].apply(mu)) * mu;
                defuzzedAlpha +=
                        alphaMemberships[alphaRuleSet[i][k]] * mu;
            }
        }

        // Scale normalized values to actual
        alpha = defuzzedAlpha;
        Kp = (KpMax - KpMin) * KpNormal + KpMin;
        Kd = (KdMax - KdMin) * KdNormal + KdMin;
        Ki = (Kp * Kp) / (alpha * Kd);
    }

    private Function<Double, Double>[] generateFuzzifierMemberships(double min, double max) {
        double gap = (max - min) / 6;
        double width = 2 * gap;

        Function<Double, Double>[] memberships = new Function[7];

        // NB and PB fuzzy sets
        memberships[0] = plateauMembership(min, -gap);
        memberships[6] = plateauMembership(max, gap);

        for (int i = 1; i <= 5; i++) {
            memberships[i] = triangularMembership(min + i * gap, width);
        }

        return memberships;
    }

//    private Function<Double, Double>[]
//    generateFuzzifierMemberships(double min, double max, double width, double setCount) {
//        double gap = (max - min) / (setCount - 1);
//
//        Function<Double, Double>[] memberships = new Function[7];
//
//        // NB and PB fuzzy sets
//        memberships[0] = plateauMembership(min, -width/2);
//        memberships[6] = plateauMembership(max, width/2);
//
//        for (int i = 1; i <= 5; i++) {
//            memberships[i] = triangularMembership(min + i * gap, width);
//        }
//
//        return memberships;
//    }

    @NonNull
    @Contract(pure = true)
    private Function<Double, Double> triangularMembership(double center, double width) {
        return (x) -> {
            if(Math.abs(x - center) >= width/2) {
                return (double) 0;
            } else {
                return -(2/width) * Math.abs(x -center) +1;
            }
        };
    }

    @NonNull
    @Contract(pure = true)
    private Function<Double, Double> plateauMembership(double center, double decay) {
        if(decay < 0) {
            return (x) -> {
                if(x <= center) {
                    return (double) 1;
                } else if(x <= center - decay) {
                    return (1 / decay) * (x - center) + 1;
                } else {
                    return (double) 0;
                }
            };
        } else if(decay > 0) {
            return (x) -> {
                if (x <= center - decay) {
                    return (double) 0;
                } else if (x <= center) {
                    return (1 / decay) * (x - center) + 1;
                } else {
                    return (double) 1;
                }
            };
        } else {
            return (x) ->{
                 if(x < center) {
                    return (double) 0;
                } else {
                    return (double) 1;
                }
            };
        }
    }

}
