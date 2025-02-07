package org.firstinspires.ftc.team13580.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team13580.RobotHardware;

@Autonomous(name="Robot auto short park", group="Robot")
public class AutoShortPark extends LinearOpMode {
    RobotHardware robot= new RobotHardware(this);
    private ElapsedTime runtime= new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();

        //go straight to get into position
        robot.setDrivePower(0.5, -0.5, -0.5, 0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);

        //go straight for 2 squares
        robot.setDrivePower(-0.2, -0.2, -0.2, -0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.00000000001)) {
            telemetry.addData("Path", "Leg3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);
    }
}
