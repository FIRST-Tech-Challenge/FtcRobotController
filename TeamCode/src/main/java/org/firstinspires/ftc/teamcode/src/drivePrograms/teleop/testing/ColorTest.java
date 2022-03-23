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
        outtake.setServoClosed();

        boolean y_depressed2 = true;

        while (opModeIsActive() && !isStopRequested()) {

            if (!gamepad2.y) {
                y_depressed2 = true;
            }
            if (gamepad2.y && y_depressed2) {
                y_depressed2 = false;
                if (outtake.isClosed()) {
                    outtake.setServoOpen();
                } else {
                    outtake.setServoClosed();
                }
            }


            if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {
                intake.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);
                RevBlinkinLedDriver.BlinkinPattern o = outtake.getLEDPatternFromFreight();
                if (o == null || !outtake.isClosed()) {
                    leds.setPattern(defaultColor);
                } else {
                    leds.setPattern(o);
                }

            } else {
                intake.setMotorPower(0);
            }

            telemetry.addData("red:", outtake.getColor(Red));
            telemetry.addData("green:", outtake.getColor(Green));
            telemetry.addData("blue:", outtake.getColor(Blue));
            telemetry.addData("distance:", outtake.getSensorDistance());
            telemetry.addData("identity:", FreightFrenzyGameObject.identify(outtake.getRGB()));

            telemetry.update();
        }
    }
}

