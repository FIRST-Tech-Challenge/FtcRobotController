package org.firstinspires.ftc.teamcode.Components;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class HorizontalSlide {
    private DcMotorEx slideMotor;
    private final int maxPosition;
    private final int minPosition;
    private final double power;
    private final double currentLimit;
    private final Telemetry telemetry;

    public HorizontalSlide(HardwareMap hardwareMap, int maxPosition, int minPosition, double currentLimit, Telemetry telemetry) {
        this.slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");


        this.maxPosition = maxPosition;
        this.minPosition = minPosition;
        this.power = 1;
        this.currentLimit = currentLimit;
        this.telemetry = telemetry;
    }

    public void moveForward() {
        if (slideMotor.getCurrentPosition() < maxPosition) {
            slideMotor.setPower(power);
            telemetry.addData("position: ", slideMotor.getCurrentPosition());
        } else {
            stopMotor();
        }
        telemetry.update();
    }

    public void moveBackward() {
        telemetry.addData("moving", "backward");
        if (slideMotor.getCurrentPosition() > minPosition && slideMotor.getCurrent(CurrentUnit.AMPS) < currentLimit) {
            slideMotor.setPower(-power);
            telemetry.addData("position: ", slideMotor.getCurrentPosition());
            telemetry.addData("power usage: ", slideMotor.getCurrent(CurrentUnit.AMPS));
        } else {
            stopMotor();
        }
        telemetry.update();
    }

    public void resetEncoder() {
        while(slideMotor.getCurrent(CurrentUnit.AMPS) < currentLimit) {
            moveBackward();
            telemetry.addData("moving back", "to reset encoder");
            telemetry.update();
        }
        telemetry.addData("hit limit", getPos());
        telemetry.update();

        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("reset encoders", "");
        telemetry.update();
    }
    // Stops the slide
    public void stopMotor() {
        slideMotor.setPower(0);
    }

    // Check if the slide has reached the maximum position

    // Check if the slide has reached the minimum position
    public boolean isAtMin() {
        return slideMotor.getCurrentPosition() <= minPosition;
    }


    // Gets the current position of the slide
    public int getPos() {
        return slideMotor.getCurrentPosition();
    }

    // Sets the power of the slide manually
    public void setPower(double power) {
        slideMotor.setPower(power);
    }
}
