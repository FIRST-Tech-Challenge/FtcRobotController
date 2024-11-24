package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.AtTargetRange;
import static org.firstinspires.ftc.teamcode.Utilize.WrapRads;
import static org.firstinspires.ftc.teamcode.Utilize.toDegree;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Autonomous(name="Auto_Bucket", group = "In")
public class Auto_Bucket extends Robot {



    private void Init() {
        // Initialize Robot
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{0, 0, 0, 0},
                new double[]{0, 0});
        imu.resetYaw();

    }

    private void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {
        }

    }

    private void keep() {

        SetServoPos(1, LJ, RJ);
        SetServoPos(0, Claw);
        SetServoPos(0.6, Ll, Rl);
        SetServoPos(0.75, LA, RA);
        SetServoPos(0.5, ADL, ADR);

        sleep(500);

        SetServoPos(0.4, Claw);
        sleep(180);
        SetServoPos(0.35, Claw);
        SetServoPos(0, Ll, Rl);
        SetServoPos(0, LA, RA);
        SetServoPos(0.12, RC);
        SetServoPos(1, ADL, ADR);
        SetServoPos(1, LJ, RJ);
        sleep(700);
        SetServoPos(0, Claw);
        SetServoPos(0.08, LA, RA);
        SetServoPos(0.68, LJ, RJ);

    }
    private void drop() {

        SetServoPos(0, LJ, RJ);
        sleep(500);
        SetServoPos(1, LJ, RJ);


    }
    private void Turn(double degs, double stopSecond) {
        double rads = Math.toRadians(degs);
        while (opModeIsActive()) {
            double yaw   = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double error =  WrapRads(rads - yaw);
            double r     =  AtTargetRange(error, 0, Math.toRadians(30)) ? 0.2 : 0.6;
            if (error < 0) r = -r;
            MovePower(r, -r, r,-r);

            if (AtTargetRange(error, 0, 0.05)) break;
        }
        Break(stopSecond);
    }
    public void runOpMode() {
        Init();
        while (!(RS.isPressed())){
            double power = RS.isPressed()? 0 : -0.4;
            LiftPower(power);
        }
        LiftPower(0);
        LL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        WaitForStart();

        if (opModeIsActive()) {


            move(-0.85, 0.61, 205.0, new double[]{0.2, 0.2, 0.2},
                    new double[]{1.2, 0.01, 0.09, 0.0}, new double[]{0.2, 0.01, 0.01, 0.0}, new double[]{0.2, 0.01, 0.001, 0.0}, 0.0001,High_Basket);

            drop();

            move(-0.6, 0.60, 189.0, new double[]{0.4, 0.4, 0.4},
                    new double[]{2.9, 0.00001, 0.1, 0.0}, new double[]{0.4, 0.009, 0.04, 0.0}, new double[]{0.2, 0.007, 0.01, 0.0}, 0.0001,0);

            keep();

            move(-0.85, 0.61, 205.0, new double[]{0.3, 0.3, 0.3},
                    new double[]{2.2, 0.01, 0.09, 0.0}, new double[]{0.4, 0.01, 0.01, 0.0}, new double[]{0.2, 0.01, 0.001, 0.0}, 0.0001, High_Basket);

            drop();

            move(-0.86, 0.65, 180.0, new double[]{0.45, 0.4, 0.4},
                    new double[]{3.0, 0.00001, 0.01, 0.0}, new double[]{0.4, 0.01, 0.025, 0.0}, new double[]{0.2, 0.01, 0.03, 0.0}, 0.0001, 0);

            keep();

            move(-0.85, 0.61, 205.0, new double[]{0.3, 0.3, 0.3},
                    new double[]{2.2, 0.01, 0.09, 0.0}, new double[]{0.4, 0.01, 0.01, 0.0}, new double[]{0.2, 0.01, 0.001, 0.0}, 0.0001, High_Basket);

            drop();

            move(-0.93, 0.70, 160.0, new double[]{0.4, 0.4, 0.4},
                    new double[]{2.9, 0.00001, 0.015, 0.0}, new double[]{0.4, 0.007, 0.001, 0.0}, new double[]{0.2, 0.007, 0.01, 0.0}, 0.0001, 0);

            keep();

            move(-0.85, 0.61, 205.0, new double[]{0.3, 0.3, 0.3},
                    new double[]{2.2, 0.01, 0.09, 0.0}, new double[]{0.4, 0.01, 0.01, 0.0}, new double[]{0.2, 0.01, 0.001, 0.0}, 0.0001, High_Basket);

            drop();

            move( 0.7, 2.25, 90.0, new double[]{0.4, 0.4, 0.4},
                    new double[]{2.2, 0.01, 0.2, 0.0}, new double[]{0.07, 0.008, 0.01, 0.0}, new double[]{0.2, 0.01, 0.01, 0.0}, 0.0001, 650);

            SetServoPos(0, LJ, RJ); // 0.55

            sleep(1000);

            }
        }
    }

