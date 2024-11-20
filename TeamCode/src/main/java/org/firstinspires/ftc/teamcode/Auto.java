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
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{0, 0, 0, 0},
                new double[]{0, 0});
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
    public void Turn(double degs, double stopSecond) {
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
    private void Lift(double height) {
        double RJLJ_pos = height == 0 ? 0 : 0.92;
        SetServoPos(RJLJ_pos, RJ, LJ);
        while (true) {
            double  curPos     = Math.max(LL.getCurrentPosition(), RL.getCurrentPosition());
            double  Lift_Power = AtTargetRange(curPos, height, 10) ?  0 :
                    (curPos > height ? -0.2    :  1);
            LiftPower(Lift_Power);
            if (Lift_Power == 0) break;
        }
    }

    public void runOpMode() {
        Init();
        WaitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
            move(1.00, 1.0, -90.0, new double[]{0.14, 0.1, 0.1}, new double[]{2.4, 0.3, 0.01, 0.0},
                 new double[]{0.12, 0.081, 0.0, 0.0},new double[]{0.1, 0.041, 0.0, 0.0}, 0.0);
            move(0.00, 0.0, 180.0, new double[]{0.14, 0.1, 0.1}, new double[]{2.4, 0.3, 0.01, 0.0},
                 new double[]{0.12, 0.09, 0.0, 0.0},new double[]{0.1, 0.041, 0.0, 0.0}, 0.1);

            }
        }
    }
}
