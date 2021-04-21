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
            driveZiptiesWithController();
            driveBeltWithController();
            driveLauncherWithController();
            fireLauncherWithTrigger(false);

            if(driver1.isButtonPressed(Button.Y) && driver2.isButtonPressed(Button.Y)){
                shootPowerShot();
            }

            if(driver1.isButtonJustPressed(Button.DPAD_UP)){
                front = !front;
            }

            telemetry.addData("Launcher RPM", (getMotorTicksPerMinute(motorLauncher, 100)) / Constants.AM_37_TICKS_PER_ROTATION);
            telemetry.addData("IMU: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}