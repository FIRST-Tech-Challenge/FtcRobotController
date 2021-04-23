package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_2020.ResourceClasses.Button;

@TeleOp(name = "TeleOp Competition", group = "TeleOp")
public class TeleOpCompetition extends MasterTeleOp {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        while (opModeIsActive()) {

            driver1.update();
            driver2.update();

            driveMecanumWithJoysticks();
            //driveMecanumWithJoysticksReverse();
            driveZiptiesWithController();
            driveBeltWithController();
            driveLauncherWithController();
            fireLauncherWithTrigger(false);

            if (driver1.isButtonPressed(Button.DPAD_RIGHT) && driver2.isButtonPressed(Button.DPAD_RIGHT)) {
                powerShotTeleOp();
            }

            if (driver1.isButtonPressed(Button.DPAD_LEFT) && driver2.isButtonPressed(Button.DPAD_LEFT)) {
                highGoalTeleOp();
            }

            if (driver1.isButtonPressed(Button.DPAD_UP)) {
                double beltSave = motorBelt.getPower();
                double zipSave = motorBelt.getPower();
                driveBelt(1.0);
                driveZiptie(1.0);
                driveMecanum(-90,1.0,0);

                pauseMillis(500);

                driveMecanum(0,0,0);
                driveBelt(beltSave);
                driveZiptie(zipSave);
            }

            telemetry.addData("Launcher RPM", (getMotorTicksPerMinute(motorLauncher, 100)) / Constants.AM_37_TICKS_PER_ROTATION);
            telemetry.addData("IMU: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}