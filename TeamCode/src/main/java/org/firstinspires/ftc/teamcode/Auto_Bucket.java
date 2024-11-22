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

    boolean On = false;

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

        SetServoPos(0.5, Claw);
        SetServoPos(0.5, RC);
        SetServoPos(0.5, Ll , Rl);
        SetServoPos(0.5, LA , RA);
        SetServoPos(0.5, ADL , ADR);

        sleep(500);

        SetServoPos(0, Claw);
        sleep(250);
        SetServoPos(0, RC);
        SetServoPos(0, Ll , Rl);
        SetServoPos(0, LA , RA);
        SetServoPos(0, ADL , ADR);
        SetServoPos(0.5, Claw);
    }
    private void drop() {

        SetServoPos(0.5, LJ, RJ);
        sleep(200);
        SetServoPos(0, LJ, RJ);

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
    private void  LiftDown(){
        while (!(RTS.isPressed())) {
            double lift_Power = RTS.isPressed() ? 0 : -0.5;
            LiftPower( lift_Power);
        }
        LL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runOpMode() {
        Init();
        WaitForStart();
        if (opModeIsActive()) {
            move(-0.85, 0.61, 218.0, new double[]{0.4, 0.4, 0.4},
                    new double[]{2.2, 0.01, 0.09, 0.0}, new double[]{0.2, 0.01, 0.01, 0.0}, new double[]{0.2, 0.01, 0.001, 0.0}, 0.0001,0);

            //drop();

            move(-0.69, 0.67, 189.0, new double[]{0.4, 0.4, 0.4},
                    new double[]{2.9, 0.00001, 0.1, 0.0}, new double[]{0.4, 0.009, 0.04, 0.0}, new double[]{0.2, 0.007, 0.01, 0.0}, 0.0001,0);

            //keep(0);

            move(-0.85, 0.61, 218.0, new double[]{0.4, 0.4, 0.4},
                    new double[]{2.2, 0.01, 0.09, 0.0}, new double[]{0.4, 0.01, 0.01, 0.0}, new double[]{0.2, 0.01, 0.001, 0.0}, 0.0001, 0);

           // drop();

            move(-0.95, 0.62, 180.0, new double[]{0.45, 0.4, 0.4},
                    new double[]{3.0, 0.00001, 0.01, 0.0}, new double[]{0.4, 0.01, 0.025, 0.0}, new double[]{0.2, 0.01, 0.03, 0.0}, 0.0001, 0);

            //keep(0);

            move(-0.85, 0.61, 218.0, new double[]{0.4, 0.4, 0.4},
                    new double[]{2.2, 0.01, 0.09, 0.0}, new double[]{0.4, 0.01, 0.01, 0.0}, new double[]{0.2, 0.01, 0.001, 0.0}, 0.0001, 0);

            //drop();

            move(-0.92, 0.73, 150.0, new double[]{0.4, 0.4, 0.4},
                    new double[]{2.9, 0.00001, 0.015, 0.0}, new double[]{0.4, 0.007, 0.001, 0.0}, new double[]{0.2, 0.007, 0.01, 0.0}, 0.0001, 0);

            //keep(0.15);

            move(-0.85, 0.61, 218.0, new double[]{0.4, 0.4, 0.4},
                    new double[]{2.2, 0.01, 0.09, 0.0}, new double[]{0.4, 0.01, 0.01, 0.0}, new double[]{0.2, 0.01, 0.001, 0.0}, 0.0001, 0);

           // drop();

            move( 0.45, 2.25, 90.0, new double[]{0.4, 0.4, 0.4},
                    new double[]{2.2, 0.01, 0.2, 0.0}, new double[]{0.07, 0.008, 0.01, 0.0}, new double[]{0.2, 0.01, 0.01, 0.0}, 0.0001, 0);

            SetServoPos(0.05, LJ, RJ); // 0.55

            sleep(1000);

            }
        }
    }

