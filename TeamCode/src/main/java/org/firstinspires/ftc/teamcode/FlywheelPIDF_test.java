package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelPIDF_test extends OpMode {
    public DcMotorEx Turret_S;
    public static double SHOOTING_VELOCITY = 2400;
    public static double PREHEAT_VELOCITY  = 1000;

    double curTargetVelocity = PREHEAT_VELOCITY;
    public static double F = 0;
    public static double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001};

    int stepIndex = 1;

    @Override
    public void init() {
        Turret_S = hardwareMap.get(DcMotorEx.class, "Turret_S");
        Turret_S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret_S.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0, F);
        Turret_S.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop() {

        if(gamepad1.yWasPressed()) {
            if(curTargetVelocity == PREHEAT_VELOCITY) {
                curTargetVelocity = SHOOTING_VELOCITY;
            } else{curTargetVelocity = PREHEAT_VELOCITY;}
        }
        if(gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0, F);
        Turret_S.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set Velocity
        Turret_S.setVelocity(curTargetVelocity);

        double curVelocity = Turret_S.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity",curTargetVelocity);
        telemetry.addData("Current Velocity","%.2f",curVelocity);
        telemetry.addData("Error","%.2f",error);
        telemetry.addLine("------------------------");
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)",P);
        telemetry.addData("Tuning F","%.4f (D-Pad L/R)",F);
        telemetry.addData("Step Size","%.4f (B button)",stepSizes[stepIndex]);

        telemetry.update();

    }
}