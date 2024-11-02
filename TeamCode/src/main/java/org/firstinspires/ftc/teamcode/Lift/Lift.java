package org.firstinspires.ftc.teamcode.Lift;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Controllers.FeedForward;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Utils.MotionProfile;

public class Lift {
    HardwareMap hardwareMap;
    FeedForward feedForward;
    PID pid;
    int motorPower;
    int currentPosition;
    DcMotorEx liftMotorLeft;
    DcMotorEx liftMotorRight;

    public Lift(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        liftMotorLeft = hardwareMap.get(DcMotorEx.class, "liftMotorLeft");
        liftMotorRight = hardwareMap.get(DcMotorEx.class, "liftMotorRight");
        liftMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        liftMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        liftMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        liftMotorLeft.setPower(0);
        liftMotorRight.setPower(0);

        currentPosition = 0;
    }
    public Action basketOne() {
        return new Action() {
            //private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //if (!initialized) {
                //    initialized = true;
                //}
                currentPosition=liftMotorLeft.getCurrentPosition();
                pid.calculate(24, currentPosition);
                liftMotorLeft.setPower(motorPower);
                liftMotorRight.setPower(motorPower);
                return currentPosition==24;
            }
        };
    }


}
