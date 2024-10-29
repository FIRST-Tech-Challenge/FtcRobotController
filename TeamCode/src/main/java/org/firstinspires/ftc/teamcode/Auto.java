package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.AtTargetRange;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@Autonomous(name="Auto", group = "In")
public class Auto extends Robot {

    private void Init() {
        // Initialize Robot
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{0, 0, 0},
                new double[]{0.3, 0, 0.1, 0});
        imu.resetYaw();

    }

    private void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {
            Posx = 0;
            Posy = 0;
            heading = 0;
        }

    }

    public void runOpMode() {
        Init();
        WaitForStart();
        if (opModeIsActive()) {
            move(0.00, 1.0, 0 , new double[]{0.0, 0.0}, new double[]{0.04, 0.09, 0.0, 0.0},new double[]{0.035, 0.08, 0.0, 0.0});
            move(1.00, 1.0, 0 , new double[]{0.0, 0.0}, new double[]{0.04, 0.09, 0.0, 0.0},new double[]{0.035, 0.08, 0.0, 0.0});
            move(2.00, 2.0, 0 , new double[]{0.0, 0.0}, new double[]{0.04, 0.09, 0.0, 0.0},new double[]{0.035, 0.08, 0.0, 0.0});
        }

    }
}
