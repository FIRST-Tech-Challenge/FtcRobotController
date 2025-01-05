package org.firstinspires.ftc.teamcode.Components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.LinkedList;
import java.util.Objects;
import java.util.Queue;
import java.util.stream.Collectors;
import java.util.Timer;

public class ViperSlide{
    private DcMotorEx leftViper;
    private DcMotorEx rightViper;

    private Servo leftBucket;
    private Servo rightBucket;
    private Servo bucketFlap;
    private Servo leftSpecimen;
    private Servo rightSpecimen;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    double minFlipLimit = 650;
    public double holdPower = .2;

    private ElapsedTime bucketCooldownTimer;

    double lastPosition;
    private Queue<Double> pastPositions;
    private static final int positionHistorySize = 5;
    String specimenGrabberPos = "closed";




    public ViperSlide(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        leftViper = hardwareMap.get(DcMotorEx.class, "leftViper");
        rightViper = hardwareMap.get(DcMotorEx.class, "rightViper");

        leftBucket = hardwareMap.get(Servo.class, "leftBucket");
        rightBucket = hardwareMap.get(Servo.class, "rightBucket");
        bucketFlap = hardwareMap.get(Servo.class, "bucketFlapServo");

        leftSpecimen = hardwareMap.get(Servo.class, "leftSpecimen");
        rightSpecimen = hardwareMap.get(Servo.class, "rightSpecimen");



        leftViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pastPositions = new LinkedList<>();


        // initialize

        // why is this here
//        leftBucket.setPosition(1);
//        rightBucket.setPosition(0);
//
//        leftViper.setTargetPosition(-2000);
//        rightViper.setTargetPosition(2000);


        bucketRest();


        openBucket();
    }

    public void setPower(double power) {
        leftViper.setPower(-power);
        rightViper.setPower(power);
    }

    public void checkInputs(
            Float retractSpeed,
            Float extendSpeed,
            Boolean resetEncoders,
            Boolean hold,
            Boolean bucketRest,
            Boolean bucketScore,
            Boolean openBucket,
            Boolean closeBucket,
            Boolean grabSpecimen,
            Boolean releaseSpecimen,
            Boolean bucketSpecimen
//            Boolean goToPositionUp,
//            Boolean goToPositionDown
    ) {
        // Move Viper
        if (retractSpeed != 0) {
            setPower(-retractSpeed);
        }
        else if (extendSpeed != 0) {
//            if(getPos() > 3000) {
//                bucketRest();
//            }
            setPower(extendSpeed);
        }
        else {
            stop();
        }
        telemetry.addData("Left Viper Position: ", leftViper.getCurrentPosition());
        telemetry.addData("Right Viper Position: ", rightViper.getCurrentPosition());
        telemetry.addData("Viper power: ", rightViper.getCurrent(CurrentUnit.AMPS));

        // Hold Position
        if(hold) {
            holdPosition(getPos());
        }

        // Reset Encoders
        if(resetEncoders) {
            resetEncoders();
        }

        // Bucket
        if(bucketScore) {
            if(getPos() > minFlipLimit && Objects.equals(specimenGrabberPos, "closed")) {
                closeBucket();
                bucketScore();
            }
            else {
                bucketCooldownTimer = new ElapsedTime();
            }
        }

        if(bucketRest) {
            if(getPos() > 300 && Objects.equals(specimenGrabberPos, "closed")) {
                bucketRest();

            }

//            if(bucketCooldownTimer != null && bucketCooldownTimer.seconds() < 1)
//                bucketRest();
//            else
//                bucketCooldownTimer = null;
        }

        if(openBucket) {
            openBucket();
            telemetry.addData("Bucket Flap Position", bucketFlap.getPosition());
            telemetry.addData("Bucket Flap Position", "open");
        }
        else if(closeBucket) {
            closeBucket();
            telemetry.addData("Bucket Flap Position", bucketFlap.getPosition());
            telemetry.addData("Bucket Flap Position", "closed");
        }

        if(grabSpecimen) {
            specimenGrabberPos = "closed";
            leftSpecimen.setPosition(0);
            rightSpecimen.setPosition(1);
        }
        else if(releaseSpecimen) {
            specimenGrabberPos = "open";
            leftSpecimen.setPosition(0.6);
            rightSpecimen.setPosition(0.4);
        }

        if(bucketSpecimen) {
            bucketSpecimen();
        }

//        if(goToPositionUp) {
//            goToPositionTest(2000);
//        }
//
//        if(goToPositionDown) {
//            goToPositionTest(0);
//        }

    }

    public void stop() {
        leftViper.setPower(0);
        rightViper.setPower(0);
    }

    public double getPos() {
        return rightViper.getCurrentPosition();
    }

    public void resetEncoders() {
        leftViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void holdPosition(double holdPosition) {
        double holdPower = 0.6;
        double holdPowerIncrement = 0.05;
        double maxHoldPower = 1.0;
        double minHoldPower = 0.0;

        double currentPosition = getPos();
        telemetry.addData("Holding at", holdPosition);

        pastPositions.add(currentPosition);
        if (pastPositions.size() > positionHistorySize) {
            pastPositions.poll();
        }

        double averagePosition = pastPositions.stream().mapToDouble(Double::doubleValue).average().orElse(currentPosition);

        if (lastPosition > currentPosition) {
            telemetry.addData("Slide fell: " + (lastPosition - currentPosition), " since last update");
            holdPower = Math.min(holdPower + holdPowerIncrement, maxHoldPower);
        } else if (lastPosition < currentPosition) {
            telemetry.addData("Slide raised: " + (currentPosition - lastPosition), " since last update");
            holdPower = Math.max(holdPower - holdPowerIncrement, minHoldPower);
        } else {
            telemetry.addData("not moving", "since last update");
        }

        lastPosition = currentPosition;

        if (averagePosition > holdPosition) {
            leftViper.setPower(-holdPower);
            rightViper.setPower(holdPower);
            telemetry.addData("slide", holdPower);
            telemetry.addData("average position", averagePosition);
        } else if (averagePosition < holdPosition) {
            leftViper.setPower(holdPower);
            rightViper.setPower(holdPower);
            telemetry.addData("slide going up at", holdPower);
            telemetry.addData("average position", averagePosition);
        } else {
            stop();
        }

        telemetry.addData("Viper Position: ", currentPosition);
    }


    public void goToPosition(int position) {
        while (rightViper.getCurrentPosition() < position) {
            setPower(1);
        }
        stop();
    }

    public void goToPositionTest(int position) {

        leftViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

//        if (position < rightViper.getCurrentPosition()) {
//            setPower(-1);
//        }
//
//        if (position > rightViper.getCurrentPosition()) {
//            setPower(1);
//        }

        setPower(1);

        stop();
    }

    public void goToRest() {
        while ((rightViper.getCurrentPosition() > 200) && (rightViper.getCurrent(CurrentUnit.AMPS) < 6)) {
            setPower(-1);
        }
        stop();
    }

    public Action goToPositionAction(int position) {
        int error = 50;
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (getPos() < position) {
                    setPower(1);
                }
                else if(getPos() > position) {
                    setPower(-1);
                }
                else {
                    stop();
                }

                packet.put("Viper Position", getPos());

                return getPos() >= (position - error) && getPos() <= (position + error);
            }
        };
    }

    public double getCurrentPower() {
        return rightViper.getCurrent(CurrentUnit.AMPS);
    }



    // bucket


    public void setBucketPosition(double leftPosition, double rightPosition) {
        leftBucket.setPosition(leftPosition);
        rightBucket.setPosition(rightPosition);
    }

    public void setBucketFlapPosition(double position) {
        bucketFlap.setPosition(position);
        telemetry.addData("Bucket Flap Position", position);
    }

    public void bucketRest() {
        setBucketPosition(0, 1);
    }

    public void bucketScore() {
        setBucketPosition(1, 0);
    } // .49, .51

    public void bucketSpecimen() {
        setBucketPosition(.2, .8);
    }

    public void closeBucket() {
        setBucketFlapPosition(.55);
    }

    public void openBucket() {
        setBucketFlapPosition(.4);
    }



    // specimen grabber
    public void setSpecimenGrabberPos(double position) {
        leftSpecimen.setPosition(position);
        rightSpecimen.setPosition(-position);
    }

    public void grabSpecimen() {
        setSpecimenGrabberPos(1);
    }

    public void releaseSpecimen() {
        setSpecimenGrabberPos(0);
    }
}

