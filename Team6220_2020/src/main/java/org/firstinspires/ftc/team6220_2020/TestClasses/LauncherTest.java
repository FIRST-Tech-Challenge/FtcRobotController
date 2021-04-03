package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_2020.Constants;
import org.firstinspires.ftc.team6220_2020.MasterTeleOp;
import org.firstinspires.ftc.team6220_2020.ResourceClasses.Button;

@TeleOp(name = "_TeleOp Competition_", group = "TeleOp")
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

            if(driver2.isButtonJustPressed(Button.RIGHT_BUMPER)) {
                isSlowMode = !isSlowMode;
            }

            if(driver2.isButtonJustPressed(Button.RIGHT_BUMPER)){
                front = !front;
            }

            telemetry.addData("Launcher RPM", (getMotorTicksPerMinute(motorLauncher, 100)) / Constants.AM_37_TICKS_PER_ROTATION);
            telemetry.addData("Slow Mode", isSlowMode);
            telemetry.addData("IMU: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES));
            telemetry.update();

            idle();
        }
    }
}