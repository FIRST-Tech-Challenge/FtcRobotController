package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.AtTargetRange;
import static org.firstinspires.ftc.teamcode.Utilize.WrapRads;
import static org.firstinspires.ftc.teamcode.Utilize.toDegree;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Autonomous(name="Auto", group = "In")
public class Auto extends Robot {

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

    private void FrontArm() {
        if (!On) {
            SetServoPos(1, Claw);
            SetServoPos(1, Ll , Rl);
            SetServoPos(1, LA , RA);

            On = true;
            return;
        }
        SetServoPos(0, Ll, Rl);
        SetServoPos(0, LA , RA);
        On = false;
    }

    public void runOpMode() {
        Init();
        WaitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
            move(1.00, 1.0, -90.0, new double[]{0.14, 0.1, 0.1}, new double[]{2.4, 0.3, 0.01, 0.0},
                 new double[]{0.12, 0.081, 0.0, 0.0},new double[]{0.1, 0.041, 0.0, 0.0}, 0.0);
            Turn(-90, 1);
            Turn(0, 1);
            move(0.00, 0.0, 180.0, new double[]{0.14, 0.1, 0.1}, new double[]{2.4, 0.3, 0.01, 0.0},
                 new double[]{0.12, 0.09, 0.0, 0.0},new double[]{0.1, 0.041, 0.0, 0.0}, 0.1);

            }
        }
    }
}
