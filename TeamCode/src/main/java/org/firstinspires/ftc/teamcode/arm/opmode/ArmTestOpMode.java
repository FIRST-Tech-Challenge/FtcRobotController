package org.firstinspires.ftc.teamcode.arm.opmode;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.arm.Arm;

/*
 * Simple test of motion-profiled arm autonomous operation. The arm should move *smoothly*
 * between random angles.
 */
@Autonomous
public class ArmTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        NanoClock clock = NanoClock.system();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            arm.setAngle(Arm.MAX_ANGLE * Math.random());

            double startTime = clock.seconds();
            while (!isStopRequested() && (clock.seconds() - startTime) < 5) {
                arm.update();
            }
        }
    }
}
