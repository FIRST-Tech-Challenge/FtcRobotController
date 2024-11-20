package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="HangSampleR", group = "In")
public class HangSampleR extends Robot {

    boolean On = false;

    private void Init() {
        // Initialize Robot
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{0, 0, 0},
                new double[]{0, 0, 0});
        imu.resetYaw();

    }

    private void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {
        }

    }

//    private void FrontArm() {
//        if (!On) {
//            SetServoPos(1, Claw);
//            SetServoPos(1, Ll , Rr);
//            SetServoPos(1, LA , RA);
//
//            On = true;
//            return;
//        }
//        SetServoPos(0, Ll, Rr);
//        SetServoPos(0, LA , RA);
//        On = false;
//        int x =1;
//    }


    public void runOpMode() {
        Init();
        WaitForStart();
        if (opModeIsActive()) {
//            while (opModeIsActive()) {
            move(-0.50, 1.0, 0.0, new double[]{0.14, 0.1, 0.1}, new double[]{2.4, 0.35, 0.01, 0.0},
                    new double[]{0.12, 0.081, 0.0, 0.0},new double[]{0.1, 0.041, 0.0, 0.0}, 0.1);
//                FrontArm();
            move(1.0, 0.10, -180.0, new double[]{0.14, 0.1, 0.1}, new double[]{2.4, 0.3, 0.01, 0.0},
                    new double[]{0.12, 0.09, 0.0, 0.0},new double[]{0.1, 0.041, 0.0, 0.0}, 0.3);
//                FrontArm();
            move(1.0, 0.0, -180.0, new double[]{0.14, 0.1, 0.1}, new double[]{2.4, 0.3, 0.01, 0.0},
                    new double[]{0.12, 0.09, 0.0, 0.0},new double[]{0.1, 0.041, 0.0, 0.0}, 0.1);

//            }
        }
    }
}
