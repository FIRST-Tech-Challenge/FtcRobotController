package org.firstinspires.ftc.teamcode.JackBurr.Other;

public class Range {
    public double num1, num2;

    public Range(double n1, double n2){
        this.num1 = n1;
        this.num2 = n2;
    }

    public boolean isInRange(double number){
        return number >= num1 && number <= num2;
    }

    public boolean isLeftOfRange(double number){
        return number < num1;
    }

    public boolean isRightOfRange(double number){
        return number > num2;
    }
}
