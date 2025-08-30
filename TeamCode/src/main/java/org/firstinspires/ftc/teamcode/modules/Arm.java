package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private final Servo lArm, rArm;

    // need to change later, riptide util?
    private double armAng = 0;

    public Arm(Servo lClawArm, Servo rClawArm){
        this.lArm = lClawArm;
        this.rArm = rClawArm;
    }

    public double getArmAng() {
        return (double) (lArm.getPosition() + rArm.getPosition()) /2;
        // This returns the *average* of the two arms' positions
        //     to return the 'true' position of the arm
    }
    public void setArmAng(double newAng) {
        armAng = newAng /* * conversionRatio*/; // Only if there is a conversion ratio
        // Any weird conversion shenanigans can be added later around here if you need it
        // Here's a weird ahh thing from 2024-2025 year
        //
        // double diff = (slidesDefaultAngle - slidePivotAng/divFactor /*weird conversion shenanigans*/);
        // double actualPosition = armAngleToServo((armAng + (diff * conversionRatio))); // we want positive numbers to correlate with a upward swing
        //
        // slidesDefaultAngle - That's if the slides can change angle stuff
        //     The default angle for last year was 90
        // divFactor - I literally don't know man
        //     I didn't write this, and I don't know why it was there
        //     Last year divFactor was 10
        // These values were hardcoded in the 24-25 year, just so y'all know
        lArm.setPosition(armAng);
        rArm.setPosition(armAng);
    }
}
