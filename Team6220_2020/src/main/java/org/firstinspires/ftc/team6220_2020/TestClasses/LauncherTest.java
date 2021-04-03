package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2020.Constants;
import org.firstinspires.ftc.team6220_2020.MasterTeleOp;
import org.firstinspires.ftc.team6220_2020.ResourceClasses.Button;

@TeleOp(name = "LauncherTestWithRPM", group = "TeleOp")
public class LauncherTest extends MasterTeleOp {

    @Override
    public void runOpMode() {

        double servoPos = .2;

        telemetry.addLine("Launcher RPM");

        Initialize();

        waitForStart();

        while (opModeIsActive()) {

            driver1.update();

            driveLauncherWithController();

            driveMecanumWithJoysticks();

            fireLauncherWithTrigger(false);

            telemetry.addData("Launcher RPM", (getMotorTicksPerMinute(motorLauncher, 100)) / Constants.AM_37_TICKS_PER_ROTATION);
            telemetry.update();

            idle();
        }
    }
}