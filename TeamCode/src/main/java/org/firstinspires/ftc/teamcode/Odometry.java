package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometry implements Runnable{

    private DcMotor leftEncoder, rightEncoder, auxEncoder;

    private double xCoord = 0, yCoord = 0, heading = 0;

    private final double COUNTS_PER_INCH;
    private final double encoderDistance;
    private final double auxEncoderOffset;

    public Odometry (DcMotor leftEncoder, DcMotor rightEncoder, DcMotor auxEncoder, double COUNTS_PER_INCH, double encoderDistance, double auxEncoderOffset){
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.auxEncoder = auxEncoder;

        this.COUNTS_PER_INCH = COUNTS_PER_INCH;
        this.encoderDistance = encoderDistance;
        this.auxEncoderOffset = auxEncoderOffset;
    }

    private double lastLeftPos = 0, lastRightPos = 0, lastAuxPos = 0;

    // https://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

    @Override
    public void run() {
        while(!Thread.currentThread().isInterrupted()) {
            double leftPos = leftEncoder.getCurrentPosition();
            double rightPos = rightEncoder.getCurrentPosition();
            double auxPos = auxEncoder.getCurrentPosition();

            double deltaLeft = leftPos - lastLeftPos;
            double deltaRight = rightPos - lastRightPos;
            double deltaAux = auxPos - lastAuxPos;

            lastLeftPos = leftPos;
            lastRightPos = rightPos;
            lastAuxPos = auxPos;

            //Find the heading in radians - 5225 Position Docs Equations
            double deltaHeading = (deltaLeft - deltaRight) / encoderDistance;
            heading += deltaHeading;

            double deltaX = deltaAux + (auxEncoderOffset * deltaHeading);
            double deltaY = (deltaLeft + deltaRight) / 2;

            xCoord += deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
            yCoord += deltaX * Math.sin(heading) + deltaY * Math.cos(heading);
        }
    }

    public double xCoord(){
        return xCoord;
    } //null exp

    public double yCoord(){
        return yCoord;
    }

    public double heading(){
        return Math.toDegrees(heading);
    }

    //
    public void stop() { //Stops the thread - IDK if this is needed
        Thread.currentThread().interrupt();
    }

}
