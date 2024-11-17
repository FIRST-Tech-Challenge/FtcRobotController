package org.firstinspires.ftc.teamcode.Components;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class HorizontalSlide {
    private DcMotorEx slideMotor;
    private final double power;
    private final double currentLimit;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    int maxSlidePosition = 675;


//    private final Gamepad gamepad1;
//    private final Gamepad gamepad2;


    public HorizontalSlide(OpMode opMode, double currentLimit) {
        slideMotor = opMode.hardwareMap.get(DcMotorEx.class, "slideMotor");
        this.power = 1;
        this.currentLimit = currentLimit;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
    }

    public void moveForward() {
        if (slideMotor.getCurrentPosition() < maxSlidePosition) {
            slideMotor.setPower(power);
        } else {
            stopMotor();
        }
    }

    public void moveBackward() {
        if (slideMotor.getCurrent(CurrentUnit.AMPS) < currentLimit) {
            slideMotor.setPower(-power);
        } else {
            stopMotor();
        }

    }

    public void checkInputs(Boolean extend, Boolean retract, Boolean reset) {
        if (extend) {
            moveForward();
        } else if (retract) {
            moveBackward();
        } else {
            stopMotor();
        }
        telemetry.addData("hSlide Power: ", slideMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("hSlide Position: ", slideMotor.getCurrentPosition());

        if(reset) {
            resetEncoder();
        }

    }

    public void resetEncoder() {
//        while(slideMotor.getCurrent(CurrentUnit.AMPS) < currentLimit) {
//            moveBackward();
//            telemetry.addData("moving back", "to reset encoder");
//            telemetry.update();
//        }
//        telemetry.addData("hit limit", getPos());
//        telemetry.update();

        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Stops the slide
    public void stopMotor() {
        slideMotor.setPower(0);
    }


    public int getPos() {
        return slideMotor.getCurrentPosition();
    }

    public void setPower(double power) {
        slideMotor.setPower(power);
    }
}
