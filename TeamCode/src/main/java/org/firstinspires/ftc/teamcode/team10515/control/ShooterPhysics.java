package org.firstinspires.ftc.teamcode.team10515.control;


public class ShooterPhysics {

    public double getShooterSpeed(double x, double y, double gearRatio){

        final double accelerationConstantY = -9.8;
        final double finalVelocityY = 0.0;
        final double accelerationConstantX = 0.0;
        final double finalVelocityX = 0.0;

        final double flywheelRadius = 0.06;

        // Y
        double velocityNotY = Math.sqrt(finalVelocityY-(2.0*accelerationConstantY*y));
        double time = (finalVelocityY - velocityNotY)/accelerationConstantY;

        // X
        double velocityNotX;
        if(accelerationConstantX == 0){
            velocityNotX = x/time;
        }
        else {
            velocityNotX = (x - ((1 / (2 * accelerationConstantX)) * Math.pow(time, 2)))/time;
        }

        // Exit velocity
        double velocityTotal = Math.sqrt(Math.pow(velocityNotY, 2) + Math.pow(velocityNotX, 2));

        // RPM
        double omega = velocityTotal/flywheelRadius;

        //Convert to degrees
        omega *= 60.0/(2*Math.PI);

        double flywheelRPM = omega;
        flywheelRPM *= 4; // Account for outside factors

        return (flywheelRPM/gearRatio)/6000.0;

    }

    public double getShooterAngle(double x, double y) {
        return Math.atan(y/x);
    }

}

