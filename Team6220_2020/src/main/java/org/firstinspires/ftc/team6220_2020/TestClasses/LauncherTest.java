package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2020.Constants;
import org.firstinspires.ftc.team6220_2020.MasterTeleOp;

@TeleOp(name = "LauncherTestWithRPM", group = "TeleOp")
public class LauncherTest extends MasterTeleOp {

    @Override
    public void runOpMode() {

        telemetry.addLine("Launcher RPM");

        Initialize();

        waitForStart();

        while (opModeIsActive()) {

            //driveLauncherWithController();
            double motorPower = gamepad1.left_stick_y;
            driveLauncherWithController();

            telemetry.addData("Launcher RPM", (getMotorTicksPerMinute(motorLauncher, 100)) / Constants.AM_37_TICKS_PER_ROTATION);
            telemetry.update();

            idle();
        }
    }
}