package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import static org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.ContinuousIntake.Colors.Blue;
import static org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.ContinuousIntake.Colors.Green;
import static org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.ContinuousIntake.Colors.Red;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.ContinuousIntake;
import org.firstinspires.ftc.teamcode.src.utills.TeleopTemplate;

/**
 * A Autonomous to test color sensor capabilities
 */
@TeleOp(name = "ColorTest")
@Disabled
public class ColorTest extends TeleopTemplate {
    private final RevBlinkinLedDriver.BlinkinPattern defaultColor = RevBlinkinLedDriver.BlinkinPattern.BLUE;

    public void runOpMode() throws InterruptedException {

        double[] sight = new double[3];
        this.initAll();
        leds.setPattern(defaultColor);
        // this instantiation assumes that the color sensor's name is color_sensor
        intake = new ContinuousIntake(hardwareMap, "intake_motor", "bucketServo", "color_sensor", true);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("red:", intake.getColor(Red));
            telemetry.addData("green:", intake.getColor(Green));
            telemetry.addData("blue:", intake.getColor(Blue));
            telemetry.addData("distance:", intake.getSensorDistance());

            if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {
                intake.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);
                RevBlinkinLedDriver.BlinkinPattern o = intake.getLEDPatternFromFreight();
                if (o == null) {
                    leds.setPattern(defaultColor);
                } else {
                    leds.setPattern(o);
                }

            } else {
                intake.setMotorPower(0);
            }

            sight[0] = intake.getColor(Red);
            sight[1] = intake.getColor(Green);
            sight[2] = intake.getColor(Blue);


            telemetry.addData("identity:", ContinuousIntake.gameObject.identify(sight));


            telemetry.update();
        }
    }
}

