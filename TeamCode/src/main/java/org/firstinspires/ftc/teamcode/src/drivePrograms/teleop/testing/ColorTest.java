package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import static org.firstinspires.ftc.teamcode.src.utills.enums.RGBCameraColors.Blue;
import static org.firstinspires.ftc.teamcode.src.utills.enums.RGBCameraColors.Green;
import static org.firstinspires.ftc.teamcode.src.utills.enums.RGBCameraColors.Red;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

/**
 * A Autonomous to test color sensor capabilities
 */
@TeleOp(name = "ColorTest")
@Disabled
public class ColorTest extends TeleOpTemplate {
    private final RevBlinkinLedDriver.BlinkinPattern defaultColor = RevBlinkinLedDriver.BlinkinPattern.BLUE;

    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(defaultColor);
        // this instantiation assumes that the color sensor's name is color_sensor
        waitForStart();
        intake.setServoOpen();

        boolean y_depressed2 = true;

        while (opModeIsActive() && !isStopRequested()) {

            if (!gamepad2.y) {
                y_depressed2 = true;
            }
            if (gamepad2.y && y_depressed2) {
                y_depressed2 = false;
                if (intake.isClosed()) {
                    intake.setServoOpen();
                } else {
                    intake.setServoClosed();
                }
            }


            if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {
                intake.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);
                RevBlinkinLedDriver.BlinkinPattern o = intake.getLEDPatternFromFreight();
                if (o == null || !intake.isClosed()) {
                    leds.setPattern(defaultColor);
                } else {
                    leds.setPattern(o);
                }

            } else {
                intake.setMotorPower(0);
            }

            telemetry.addData("red:", intake.getColor(Red));
            telemetry.addData("green:", intake.getColor(Green));
            telemetry.addData("blue:", intake.getColor(Blue));
            telemetry.addData("distance:", intake.getSensorDistance());
            telemetry.addData("identity:", FreightFrenzyGameObject.identify(intake.getRGB()));

            telemetry.update();
        }
    }
}

