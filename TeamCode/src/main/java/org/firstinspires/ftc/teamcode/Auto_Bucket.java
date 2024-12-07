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

        SetServoPos(0.9, RJ);
        SetServoPos(0, Claw);
        SetServoPos(0.6, Ll, Rl);
        SetServoPos(0.8, LA, RA);
        SetServoPos(0.5, ADL, ADR);

        sleep(500);

        SetServoPos(0.4, Claw);
        sleep(180);
        SetServoPos(0.35, Claw);
        SetServoPos(0, Ll, Rl);
        SetServoPos(0, LA, RA);
        SetServoPos(0.12, RC);
        SetServoPos(0, ADL, ADR);
        SetServoPos(0.9, RJ);

         On = false;

        sleep(800);
        SetServoPos(0, Claw);
        SetServoPos(0, LA, RA);
        SetServoPos(0.67, RJ);

    }
    private void drop() {

        SetServoPos(0, RJ);
        sleep(900);
        SetServoPos(1, RJ);

        On = true;

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
        SetServoPos(0.67, RJ);
        LL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Posx = 0;
        Posy = 0;

        WaitForStart();

        if (opModeIsActive()) {

            move(-0.91, 0.43, 215.0, new double[]{0.3, 0.2, 0.25},
                    new double[]{2.4, 0.0008, 0.09, 0.0}, new double[]{0.1, 0.4, 0.04, 0.0}, new double[]{0.2, 0.2, 0.07, 0.0}, 0.0001,High_Basket);

            drop();

            move(-0.5, 0.6, 180.0, new double[]{0.2, 0.2, 0.2},
                    new double[]{3.0, 0.0006, 0.12, 0.0}, new double[]{0.25, 0.4, 0.07, 0.0}, new double[]{0.35, 0.2, 0.1, 0.0}, 0.0001,0);

            keep();

            move(-0.91, 0.43, 215.0, new double[]{0.3, 0.2, 0.25},
                    new double[]{2.47, 0.0008, 0.09, 0.0}, new double[]{0.1, 0.4, 0.04, 0.0}, new double[]{0.2, 0.2, 0.07, 0.0}, 0.0001,High_Basket);


            drop();

            move(-0.935, 0.6, 180.0, new double[]{0.2, 0.2, 0.2},
                    new double[]{3.0, 0.0006, 0.12, 0.0}, new double[]{0.25, 0.4, 0.07, 0.0}, new double[]{0.35, 0.2, 0.1, 0.0}, 0.0001, 0);

            keep();

            move(-0.91, 0.43, 215.0, new double[]{0.3, 0.2, 0.25},
                    new double[]{2.47, 0.0008, 0.09, 0.0}, new double[]{0.1, 0.4, 0.04, 0.0}, new double[]{0.2, 0.2, 0.07, 0.0}, 0.0001,High_Basket);

            drop();

            move(-0.765, 0.75, 143.0, new double[]{0.2, 0.2, 0.2},
                    new double[]{3.0, 0.0006, 0.12, 0.0}, new double[]{0.25, 0.4, 0.07, 0.0}, new double[]{0.35, 0.2, 0.1, 0.0}, 0.0001, 0);

            SetServoPos(0.9, RC);
            keep();

            move(-0.91, 0.4, 215.0, new double[]{0.3, 0.2, 0.25},
                    new double[]{2.43, 0.0006, 0.09, 0.0}, new double[]{0.1, 0.4, 0.04, 0.0}, new double[]{0.2, 0.2, 0.07, 0.0}, 0.0001,High_Basket);


            drop();

            move( 0.5, 2.25, 90.0, new double[]{0.2, 0.2, 0.2},
                    new double[]{2.8, 0.0006, 0.12, 0.0}, new double[]{0.2, 0.4, 0.04, 0.0}, new double[]{0.45, 0.2, 0.12, 0.0}, 0.0001, 650);

            SetServoPos(0, RJ);


            sleep(1000);

            }
        }
    }

