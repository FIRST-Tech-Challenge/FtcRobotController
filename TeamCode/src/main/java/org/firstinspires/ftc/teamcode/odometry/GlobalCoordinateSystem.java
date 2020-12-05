package org.firstinspires.ftc.teamcode.odometry;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class GlobalCoordinateSystem implements Runnable {

        DcMotor leftEncoder, rightEncoder, middleEncoder;

        boolean isRunning = true;

        double leftEncoderPosition, rightEncoderPosition, middleEncoderPosition;
        double changeInOrientation;
        double OLDleftEncoderPosition, OLDrightEncoderPosition, OLDmiddleEncoderPosition;

        double globalx, globaly, robotOrientation;

        double enconderWheelDistance;
        double middleEncoderTickOffset;

        int sleepTime;

        //creates text files on phone that are used in odometry
        File sideWheelSeparationFile = AppUtil.getInstance().getSettingsFile("sideWheelsSeparationFile.txt");
        File middleTickOffsetFile = AppUtil.getInstance().getSettingsFile("middleTickOffsetFile.txt");

        //contructor for class
        public GlobalCoordinateSystem(DcMotor leftEncoder, DcMotor rightEncoder, DcMotor middleEncoder, double TICKS_PER_INCH, int threadSleepDelay)
        {
            this.leftEncoder = leftEncoder;
            this.rightEncoder = rightEncoder;
            this.middleEncoder = middleEncoder;
            sleepTime = threadSleepDelay;

            //getting stuff from text file
            enconderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(sideWheelSeparationFile).trim()) * TICKS_PER_INCH;
            middleEncoderTickOffset = Double.parseDouble(ReadWriteFile.readFile(middleTickOffsetFile).trim());
        }

        //method
        public void positionUpdate(){
            leftEncoderPosition = leftEncoder.getCurrentPosition();
            rightEncoderPosition = rightEncoder.getCurrentPosition();

            //constantly updated i think
            double leftChange = leftEncoderPosition - OLDleftEncoderPosition;
            double rightChange = rightEncoderPosition - OLDrightEncoderPosition;

            //finding the robot orientation currently
            changeInOrientation = (leftChange - rightChange) / enconderWheelDistance;
            robotOrientation += changeInOrientation;

            //just finidng the horizontal change
            middleEncoderPosition = middleEncoder.getCurrentPosition();
            double rawHorizontalChange = middleEncoderPosition - OLDrightEncoderPosition;
            double horizontalChange = rawHorizontalChange - (changeInOrientation*middleEncoderTickOffset);

            double sides = (rightChange + leftChange)/2; //getting average
            double frontBack = horizontalChange;

            //updating position
            //idk why they're doing sin and cos but yeah
            //probaly go back and figure out
            globalx = sides*Math.sin(robotOrientation) + frontBack*Math.cos(robotOrientation);
            globaly = sides*Math.cos(robotOrientation) + frontBack*Math.sin(robotOrientation);

            //updating position
            OLDrightEncoderPosition = rightEncoderPosition;
            OLDleftEncoderPosition = leftEncoderPosition;
            OLDmiddleEncoderPosition = middleEncoderPosition;
        }

        //bunch of methods
        public double returnXCoordinate(){return globalx; }
        public double returnYCoordinate(){return globaly; }
        public double returnOrientation(){return Math.toDegrees(robotOrientation)%360;}

        public void stop(){isRunning = false;}


        @Override
    public void run() {
            while(isRunning){
                //constantly updating
                positionUpdate();
            }
            try{
                Thread.sleep(sleepTime);
            }catch(InterruptedException e){
                e.printStackTrace();
            }
    }
    }

