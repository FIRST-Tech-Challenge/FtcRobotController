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

            move(-0.15, 1.19, 0.0, new double[]{0.2, 0.2, 0.2},
                    new double[]{2.9, 0.00001, 0.1, 0.0}, new double[]{0.4, 0.009, 0.04, 0.0}, new double[]{0.1, 0.007, 0.01, 0.0}, 0.0001, High_Chamber);

            move(-0.15, 1.19, 0.0,  new double[]{0.05, 0.1, 0.1},
                    new double[]{2.9, 0.00001, 0.1, 0.0}, new double[]{0.2, 0.004, 0.35, 0.0}, new double[]{0.1, 0.007, 0.01, 0.0}, 0, 1100);

            move(1.0, 1.19, 0.0,  new double[]{0.1, 0.3, 0.1},
                    new double[]{2.9, 0.00001, 0.1, 0.0}, new double[]{0.45, 0.0009, 0.055, 0.0}, new double[]{0.2, 0.007, 0.05, 0.0}, 0, 0);

            move(1.0, 2.2, 0.0, new double[]{0.01, 0.1, 0.3},
                    new double[]{2.9, 0.00001, 0.1, 0.0}, new double[]{0.45, 0.003, 0.08, 0.0}, new double[]{0.1, 0.0003, 0.0085, 0.0}, 0, 0);



            move(1.4, 0.55, 0.0, new double[]{0.01, 0.05, 0.1},
                    new double[]{2.0, 0.08, 0.1, 0.0}, new double[]{0.7, 0.1, 0.015, 0.0},  new double[]{0.09, 0.0001, 0.01, 0.0}, 0, 0);

            move(1.4, 2.2, 0.0, new double[]{0.01, 0.1, 0.1},
                    new double[]{2.0, 0.08, 0.1, 0.0}, new double[]{0.1, 0.01, 0.0009, 0.0}, new double[]{0.09, 0.0001, 0.01, 0.0}, 0, 0);

            move(1.4, 0.55, 0.0, new double[]{0.01, 0.05, 0.1},
                    new double[]{2.0, 0.08, 0.1, 0.0}, new double[]{0.7, 0.1, 0.015, 0.0},  new double[]{0.09, 0.0001, 0.01, 0.0}, 0, 0);



            move(1.2, 0.4, 180.0, new double[]{0.2, 0.1, 0.0},
                    new double[]{2.1, 0.1, 0.3, 0.0}, new double[]{0.05, 0.07, 0.001, 0.0}, new double[]{0.08, 0.00001, 0.009, 0.0}, 0.001, 0);

            move(1.2, 0.0, 180.0, new double[]{0.05, 0.1, 0.2},
                    new double[]{2.0, 0.1, 0.3, 0.0}, new double[]{0.09, 0.07, 0.03, 0.0}, new double[]{0.16, 0.00001, 0.012, 0.0}, 0.001, 0);

            move(-0.35, 1.15, 0.0, new double[]{0.12, 0.1, 0.1},
                    new double[]{3.0, 0.0001, 0.04, 0.0}, new double[]{0.09, 0.001, 0.001, 0.0}, new double[]{0.06, 0.001, 0.002, 0.0}, 0.0001, High_Chamber);

            move(-0.35, 1.15, 0.0, new double[]{0.05, 0.1, 0.1},
                    new double[]{3.0, 0.0001, 0.02, 0.0}, new double[]{0.04, 0.5, 0.02, 0.0}, new double[]{0.06, 0.001, 0.002, 0.0}, 0.0001, 1100);

            move(1.2, 0.4, 180.0, new double[]{0.05, 0.1, 0.1},
                    new double[]{2.0, 0.1, 0.3, 0.0}, new double[]{0.09, 0.07, 0.001, 0.0}, new double[]{0.08, 0.00001, 0.009, 0.0}, 0.001, 0);

            move(1.2, 0.0, 180.0, new double[]{0.05, 0.1, 0.1},
                    new double[]{2.0, 0.1, 0.3, 0.0}, new double[]{0.09, 0.01, 0.06, 0.0}, new double[]{0.16, 0.00001, 0.012, 0.0}, 0.001, 0);

            move(-0.45, 1.2, 0.0, new double[]{0.05, 0.1, 0.1},
                    new double[]{3.0, 0.0001, 0.04, 0.0}, new double[]{0.09, 0.001, 0.001, 0.0}, new double[]{0.06, 0.0001, 0.002, 0.0}, 0.0001, High_Chamber);

            move(-0.45, 1.2, 0.0, new double[]{0.05, 0.1, 0.1},
                    new double[]{3.0, 0.0001, 0.02, 0.0}, new double[]{0.09, 0.0002, 0.02, 0.0}, new double[]{0.09, 0.0001, 0.002, 0.0}, 0.0001, 1100);

////            move(1.0, 1.0, 0.4, 180.0, new double[]{0.25, 0.5, 0.5},
////                    new double[]{2.0, 0.1, 0.3, 0.0}, new double[]{0.12, 0.01, 0.005, 0.0}, new double[]{0.1, 0.1, 0.01, 0.0}, 0.001, 0);
////
////            move(1.0, 1.0, 0.07, 180.0, new double[]{0.25, 0.55, 0.55},
////                    new double[]{2.0, 0.1, 0.3, 0.0}, new double[]{2.0, 0.01, 0.06, 0.0}, new double[]{0.17, 0.1, 0.01, 0.0}, 0.001, 0);
////
////            move(1.0, -0.45, 1.19, 0.0, new double[]{1.0, 0.5, 0.5},
////                    new double[]{2.0, 0.1, 0.2, 0.0}, new double[]{0.17, 0.1, 0.005, 0.0}, new double[]{0.2, 0.1, 0.009, 0.0}, 0.0001, 1800);
////
////            move(1.0, -0.45, 1.19, 0.0, new double[]{1.0, 0.5, 0.5},
////                    new double[]{2.0, 0.1, 0.2, 0.0}, new double[]{0.3, 0.2, 0.02, 0.0}, new double[]{0.17, 0.2, 0.02, 0.0}, 0.0001, 1100);
//            move(1.5, 0.3, 0.0, new double[]{0.05, 0.1, 0.1},
//                    new double[]{2.0, 0.1, 0.3, 0.0}, new double[]{0.15, 0.0001, 0.007, 0.0}, new double[]{0.15, 0.0001, 0.007, 0.0}, 0.001, 0);

        }
    }
}

