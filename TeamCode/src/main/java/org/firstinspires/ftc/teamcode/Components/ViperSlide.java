package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ViperSlide {
    private DcMotorEx leftViper;
    private DcMotorEx rightViper;

    private Servo leftBucket;
    private Servo rightBucket;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    int holdPosition;
    double minFlipLimit = 650;
    public double holdPower = .2;


    public ViperSlide(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        leftViper = hardwareMap.get(DcMotorEx.class, "leftViper");
        rightViper = hardwareMap.get(DcMotorEx.class, "rightViper");

        leftBucket = hardwareMap.get(Servo.class, "leftBucket");
        rightBucket = hardwareMap.get(Servo.class, "rightBucket");

        leftViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // initialize
        leftBucket.setPosition(1);
        rightBucket.setPosition(0);
    }

    public void setPower(double power) {
        leftViper.setPower(power);
        rightViper.setPower(-power);
    }

    public void checkInputs(Float retractSpeed,
                            Float extendSpeed,
                            Boolean resetEncoders,
                            Boolean hold,
                            Boolean bucketRest,
                            Boolean bucketScore
    ) {
        // Move Viper
        if (retractSpeed != 0) {
            setPower(retractSpeed);
        }
        else if (extendSpeed != 0) {
            setPower(-extendSpeed);
        }
        else {
            stop();
        }
        telemetry.addData("Viper Position: ", rightViper.getCurrentPosition());
        telemetry.addData("Viper power: ", rightViper.getCurrent(CurrentUnit.AMPS));

        // Hold Position
        if(hold) {
            holdPosition(holdPower);
        }

        // Reset Encoders
        if(resetEncoders) {
            resetEncoders();
        }

        // Bucket
        if(bucketRest) {
            bucketRest();
        }
        else if(bucketScore && (getPos() > minFlipLimit)) {
            bucketReceive();
        }
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

    public void holdPosition(double power) {
        holdPosition = rightViper.getCurrentPosition();

        if(rightViper.getCurrentPosition() > holdPosition) {
            leftViper.setPower(-power);
            rightViper.setPower(power);
        } else if(rightViper.getCurrentPosition() < holdPosition) {
            leftViper.setPower(power);
            rightViper.setPower(-power);
        } else {
            stop();
        }
    }

    // bucket


    public void setBucketPosition(double leftPosition, double rightPosition) {
        leftBucket.setPosition(leftPosition);
        rightBucket.setPosition(rightPosition);
    }

    public void bucketRest() {
        setBucketPosition(1, 0);
    }

    public void bucketReceive() {
        setBucketPosition(.55, .45);
    }


}