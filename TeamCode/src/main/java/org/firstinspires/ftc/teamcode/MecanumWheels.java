package org.firstinspires.ftc.teamcode;


import java.lang.*;

public class MecanumWheels {

    final private int FrontLeft = 0;
    final private int FrontRight = 1;
    final private int RearLeft = 2;
    final private int RearRight = 3;

    private double move_x;
    private double move_y;
    private double turn;

    double[] wheelPowers = new double[4];

    // Override Default constructor
    public MecanumWheels() {
        move_x =0;
        move_y =0;
        turn = 0;

        for (int i = 0; i < wheelPowers.length; i++) {
            wheelPowers[i] = 0.0;
        }
    }

    private void CalclulatePower() {


        double r = Math.hypot(move_x, move_y);
        double robotAngle = Math.atan2(move_y, move_x) - Math.PI / 4;
        double rightX = turn;


        wheelPowers[FrontLeft] = r * Math.cos(robotAngle) - rightX;
        wheelPowers[FrontRight] = r * Math.sin(robotAngle) + rightX;
        wheelPowers[RearLeft] = r * Math.sin(robotAngle) - rightX;
        wheelPowers[RearRight] = r * Math.cos(robotAngle) + rightX;

        // Adjust maximum power from -1.0 to 1.0
        double absMax = 0.0;
        for (int i = 0; i < wheelPowers.length; i++) {
            if (Math.abs(wheelPowers[i]) > absMax) {
                absMax = Math.abs(wheelPowers[i]);
            }
        }

        if (absMax > 1.0) {
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] = wheelPowers[i] / absMax;
            }
        }
    }

    public void UpdateInput(double a, double b, double c) {
        this.move_x = a;
        this.move_y = b;
        this.turn = c;
        CalclulatePower();
    }



    public double getFrontLeftPower() {
        return wheelPowers[FrontLeft];
    }

    public double getFrontRightPower() {
        return wheelPowers[FrontRight];
    }

    public double getRearLeftPower() {
        return wheelPowers[RearLeft];
    }

    public double getRearRightPower() {
        return wheelPowers[RearRight];
    }



    public void print(Boolean header) {
        if (header) {
            System.out.printf("move_x, move_y, turn, FrontLeft, " +
                    "FrontRight, RearLeft, RearRight%n");
        }
        System.out.printf("%6.2f, %6.2f, %7.2f, %9.2f, %10.2f, %8.2f, %9.2f%n",
                move_x, move_y, turn,
                wheelPowers[FrontLeft], wheelPowers[FrontRight],
                wheelPowers[RearLeft], wheelPowers[RearRight]);
    }

    public static void main(String[] args) {
        double JoyPos[] = { 1.0, 0.0, -1.0 };
        //double JoyPos[] = { 1.0, 0.75, 0.5, 0.25, 0.0, -0.5, -0.75, -1.0 };
        MecanumWheels dt = new MecanumWheels();
        dt.print(true);

        for (int left_x = 0; left_x < JoyPos.length; left_x++) {
            for (int left_y = 0; left_y < JoyPos.length; left_y++) {
                for (int right_x = 0; right_x < JoyPos.length; right_x++) {
                    dt.UpdateInput(JoyPos[left_x], JoyPos[left_y], JoyPos[right_x]);
                    dt.print(false);
                }
            }
        }
    }

}
