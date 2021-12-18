package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.Utills.TeleopTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.ContinuousIntake;

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

            telemetry.addData("red:", intake.getColor("red"));
            telemetry.addData("green:", intake.getColor("green"));
            telemetry.addData("blue:", intake.getColor("blue"));
            telemetry.addData("distance:", intake.getCloseDistance());

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

            sight[0] = intake.getColor("red");
            sight[1] = intake.getColor("green");
            sight[2] = intake.getColor("blue");


            telemetry.addData("identity:", ContinuousIntake.gameObject.identify(sight));


            telemetry.update();
        }
    }
}

