package org.firstinspires.ftc.teamcode.bots;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.Date;

public class ShooterBot extends IntakeBot {
    public DcMotor shooter = null;
    public Servo pusher = null;

    long currentTime = 0;
    long lastTime = 0;
    long timeDifference = 0;

    double currentPosition = 0;
    double lastPosition = 0;
    double positionDifference = 0;

    public double currentShooterSpeed = 1;

    //change these values to control what speed the shooter spins around
    public double highShooterSpeedThreshold = 1.385; //1.4
    public double lowShooterSpeedThreshold = 1.38; //1.395
    public double setShooterSpeed = 1.591;

    //the two speeds the shooter switches between to control itself
    public double highShooterSpeed = -0.7;
    public double lowShooterSpeed = -0.285;

    //two positions of the pusher servo
    final double pusherRetracted = 0.5;
    final double pusherPushing = 0.585;//0.605

    boolean shooterIsOn = false;
    public boolean isAuto = true;

    OutputStreamWriter shooterWriter;

    //MiniPID shooterPID = new MiniPID(0.6, 0.2, 0.5);

    public ShooterBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        shooter = hwMap.get(DcMotor.class, "Shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pusher = hwMap.get(Servo.class, "Pusher");
        pusher.setPosition(pusherRetracted);
        //shooterPID.setOutputLimits(0.9);
        try {
            shooterWriter = new FileWriter("/sdcard/FIRST/shooterlog_" + java.text.DateFormat.getDateTimeInstance().format(new Date()) + ".csv", true);
        } catch (IOException e) {
            throw new RuntimeException("shooter log file writer open failed: " + e.toString());
        }
    }

    public void toggleShooter(boolean input) {
        if (!isDown) {
            shooterIsOn = true;
        } else {
            shooterIsOn = false;
        }
    }

    public void spinShooter() {
        if (!isAuto) {
            highShooterSpeedThreshold = 1.325;
            lowShooterSpeedThreshold = 1.323;
            highShooterSpeed = -0.65;
            lowShooterSpeed = -0.1;
        }
        if (shooterIsOn) {
            //calculate difference in time between last and current cycle
            currentTime = System.currentTimeMillis();
            timeDifference = currentTime - lastTime;
            //calculate difference in position between last and current cycle
            currentPosition = shooter.getCurrentPosition();
            positionDifference = Math.abs(currentPosition - lastPosition);
            //calculate current shooter speed
            currentShooterSpeed = positionDifference / (double)timeDifference;
            //check if current speed is less than or high than the two thresholds
            //double adjustSpeed = shooterPID.getOutput(currentShooterSpeed, setShooterSpeed);
            //shooter.setPower(- adjustSpeed);
            if (currentShooterSpeed < lowShooterSpeedThreshold) {
                //increase shooter power to compensate
                shooter.setPower(highShooterSpeed);
            }
            if (currentShooterSpeed > highShooterSpeedThreshold) {
                //decrease shooter power to compensate
                shooter.setPower(lowShooterSpeed);
            }
            //save current time and position for next cycle
            lastTime = currentTime;
            lastPosition = currentPosition;
            opMode.telemetry.addData("Shooter speed", currentShooterSpeed);
            // opMode.telemetry.addData("Position Difference", positionDifference);
            // opMode.telemetry.addData("Time Difference", (double)timeDifference);
            //opMode.telemetry.addData("Current Position", currentPosition);
            opMode.telemetry.update();
            try {
                RobotLog.d("shooterWriter.write");
                shooterWriter.write(String.format("%d, %f\n", currentTime, currentShooterSpeed));
                //                //shooterWriter.write(String.format("%d, %f, %f\n", currentTime, currentShooterSpeed, adjustSpeed));
            } catch (IOException e) {
                throw new RuntimeException("shooter log file writer write failed: " + e.toString());
            }
        } else {
            shooter.setPower(0);
        }
    }

    public void waitForThreshold() {
        while (currentShooterSpeed < lowShooterSpeedThreshold || currentShooterSpeed > highShooterSpeedThreshold) {
            onLoop(20, "wait between shots");
        }
    }
    public void waitForThreshold(double low, double high) {
        while (currentShooterSpeed < low || currentShooterSpeed > high) {
            onLoop(20, "wait between shots");
        }
    }

    public void launchRing(boolean rightBumper) {
        if (rightBumper) {
            pusher.setPosition(pusherPushing);
            sleep(300, "launch ring 1"); //700 ms for normal servo
            pusher.setPosition(pusherRetracted);
            sleep(300, "launch ring 2"); //700 ms for normal servo
        }
    }

//    public void shootPowerShots() {
//        double originalAngle;
//        originalAngle = startAngle;
//
//        // distance (in mm) = revolution * pi * diameter (100 mm)
//        int distanceTicks = 35000;
//        int powerShotSpacing = 9000;
//        int startingPosition = horizontal.getCurrentPosition();
//        double maxPower = 0.4;
//
//        MiniPID pid = new MiniPID(0.025, 0.005, 0.015);
//        pid.setOutputLimits(maxPower);
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        double angle;
//        angle = getAngle();
//        double adjustPower = pid.getOutput(angle, originalAngle);
//        int currentPosition = horizontal.getCurrentPosition();
//        while (Math.abs(currentPosition - startingPosition) < distanceTicks) {
//            onLoop(50, "gyro drive 1");
//
//            if (Math.abs(currentPosition - startingPosition) > (powerShotSpacing - powerShotSpacing)) {
//                launchRing(true);
//                powerShotSpacing += powerShotSpacing;
//            }
//
//            leftFront.setPower(- maxPower - adjustPower);
//            rightFront.setPower(+ maxPower + adjustPower * highRPMToLowRPM);
//            leftRear.setPower(+ maxPower - adjustPower * highRPMToLowRPM);
//            rightRear.setPower(- maxPower + adjustPower);
//
//            //onLoop(30, "gyro drive 2");
//            angle = getAngle();
//            adjustPower = pid.getOutput(angle, originalAngle);
//            currentPosition = horizontal.getCurrentPosition();
//        }
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftFront.setPower(0);
//        rightFront.setPower(0);
//        leftRear.setPower(0);
//        rightRear.setPower(0);
//        sleep(500, "after gyro wait");
//    }

    protected void onTick(){
        spinShooter();
        super.onTick();
    }

    public void close(){
        try {
            RobotLog.d("shooter log Writer.close");
            shooterWriter.close();
        } catch (IOException e) {
            throw new RuntimeException("shooter log file writer close failed: " + e.toString());
        }
        super.close();
    }

}
