package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideIntake {
    private DcMotorEx slideMotor = null;

    public class SlideOut implements Action{
        private boolean initialized = false;
        private double targetPosition = 0.0;
        public SlideOut (double targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetryPacket.put("Entering Slide out", targetPosition);
            // powers on motor, if it is not on
            if (!initialized) {
                slideMotor.setPower(-0.6);
                initialized = true;
            }

            double slideMotorPos = slideMotor.getCurrentPosition();
            telemetryPacket.put("Slide Motor Position ", slideMotorPos);

            if (slideMotorPos > targetPosition) {
                return true;
            } else {
                slideMotor.setPower(0);
                return false;
            }
        }
    }

    public class SlideIn implements Action{
        private boolean initialized = false;
        private double targetPosition = 0.0;
        public SlideIn (double targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // powers on motor, if it is not on
            if (!initialized) {
                slideMotor.setPower(0.6);
                initialized = true;
            }

            double slideMotorPos = slideMotor.getCurrentPosition();
            telemetryPacket.put("Slide Motor Position ", slideMotorPos);

            if (slideMotorPos < targetPosition) {
                return true;
            } else {
                slideMotor.setPower(0);
                return false;
            }
        }
    }

    public SlideIntake(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideIntake");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Action slideOut(double targetPosition) {
        return new SlideOut(targetPosition);
    }

    public Action slideIn(double targetPosition) {
        return new SlideIn(targetPosition);
    }

}
