package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.AtTargetRange;
import static org.firstinspires.ftc.teamcode.Utilize.WrapRads;
import static org.firstinspires.ftc.teamcode.Utilize.toDegree;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Autonomous(name="Auto_Sample", group = "In")
public class Auto_Sample extends Robot {
    int  i = 0;



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
        SetServoPos(0.45, RC);
        SetServoPos(0, Claw);
        SetServoPos(0.6, Ll, Rl);
        SetServoPos(0.8, LA, RA);
        SetServoPos(0.5, ADL, ADR);

        sleep(400);
        SetServoPos(0.35, Claw);
        sleep(150);
        SetServoPos(0.7, LA, RA);
    }

    private void place() {
        SetServoPos(0, Claw);
        sleep(100);

        SetServoPos(0, Ll, Rl);
        SetServoPos(0, LA, RA);
        SetServoPos(0, ADL, ADR);
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

            move(-0.15, 1.22, 0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.0008, 0.09, 0.0}, new double[]{0.1, 0.4, 0.04, 0.0}, new double[]{0.1, 0.1, 0.01, 1.0}, 0.000001,High_Chamber);

            move(-0.15, 1.22, 0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.0008, 0.09, 0.0}, new double[]{0.1, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0000001,1100);

            move(0.735, 0.91, -125.0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.0001, 0.12, 1.0}, new double[]{0.15, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0000001,0);

            keep();

            move(1.1, 0.7, -45.0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.0001, 0.12, 1.0}, new double[]{0.15, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0000001,0);

            place();

            move(1.1, 0.91, -125.0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.0001, 0.12, 1.0}, new double[]{0.15, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0000001,0);

            keep();

            move(1.1, 0.7, -45.0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.0001, 0.12, 1.0}, new double[]{0.15, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0000001,0);

            place();

            move(1.2, 0.2, -180.0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.0001, 0.12, 0.0}, new double[]{0.15, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0000001,0);

            move(1.2, -0.045, -180.0, new double[]{1, 1, 1},
                    new double[]{2.9, 0.0001, 0.12, 0.0}, new double[]{0.1, 0.4, 0.04, 0.0}, new double[]{0.5, 0.1, 0.01, 0.0}, 0.0000001,0);

            move(-0.45, 1.22, 0.0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.0001, 0.12, 0.0}, new double[]{0.2, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0001,High_Chamber);

//            move(-0.35, 1.19, 0.0, new double[]{0.2, 0.2, 0.2},
//                    new double[]{2.4, 0.006, 0.12, 0.0}, new double[]{0.2, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0001,1300);


            move(1.2, 0.2, 180.0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.0001, 0.12, 0.0}, new double[]{0.15, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0000001,0);

            move(1.2, -0.035, 180.0, new double[]{1, 0.2, 1},
                    new double[]{2.9, 0.0001, 0.12, 0.0}, new double[]{0.1, 0.4, 0.04, 0.0}, new double[]{0.5, 0.1, 0.01, 0.0}, 0.0000001,0);

            move(-0.35, 1.22, 0.0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.0001, 0.12, 0.0}, new double[]{0.2, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0001,High_Chamber);

//            move(-0.45, 1.19, 0.0, new double[]{0.2, 0.2, 0.2},
//                    new double[]{2.4, 0.006, 0.12, 0.0}, new double[]{0.2, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0001,1300);


            move(1.2, 0.2, 180.0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.0006, 0.12, 0.0}, new double[]{0.15, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0000001,0);

            move(1.2, -0.045, 180.0, new double[]{1, 1, 1},
                    new double[]{2.9, 0.0006, 0.12, 0.0}, new double[]{0.1, 0.4, 0.04, 0.0}, new double[]{0.5, 0.1, 0.01, 0.0}, 0.0000001,0);

            move(-0.25, 1.22, 0.0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.0006, 0.12, 0.0}, new double[]{0.2, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0001,High_Chamber);

            move(-0.25, 1.19, 0.0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.006, 0.12, 0.0}, new double[]{0.2, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0001,1300);
            move(0.45, 0.8, -45.0, new double[]{1, 1, 1},
                    new double[]{2.4, 0.006, 0.12, 0.0}, new double[]{0.2, 0.4, 0.04, 0.0}, new double[]{0.15, 0.1, 0.02, 0.0}, 0.0001,0);
        }
    }
}

