package org.firstinspires.ftc.teamcode.CompBotSimplified;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

@Autonomous
@Disabled
public class YesMode extends LinearOpMode {
    CompBotHWSimplified r = new CompBotHWSimplified();
    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);

        waitForStart();

        double dForward = 48, dStrafe = 0, sForward = 0.10, sStrafe = 0;

        r.positionControl();
        double[] positions = {dForward+dStrafe, dForward-dStrafe, dForward-dStrafe, dForward+dStrafe};
        r.setTargetDist(positions);
        double initialHeading = r.imu.getHeading(), error;
        while(!(r.fl.atTargetPosition() || r.fr.atTargetPosition() || r.bl.atTargetPosition() || r.br.atTargetPosition())) {
            error = r.imu.getHeading() - initialHeading;
            r.m.driveRobotCentric(sStrafe,sForward, 0);
            telemetry.addData("calc positions",Arrays.toString(positions));
            telemetry.addData("error",error);
            double[] actualPositions = {r.fl.getCurrentPosition(), r.fr.getCurrentPosition(), r.bl.getCurrentPosition(), r.br.getCurrentPosition()};
            telemetry.addData("actual", Arrays.toString(actualPositions));
            telemetry.update();
        }
        r.m.stop();
        r.rawPower();
    }
}
