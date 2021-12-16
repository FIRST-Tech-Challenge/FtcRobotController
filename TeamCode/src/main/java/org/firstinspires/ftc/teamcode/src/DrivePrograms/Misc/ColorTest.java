package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.Utills.TeleopTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.ContinuousIntake;

@TeleOp(name = "ColorTest")
@Disabled
public class ColorTest extends TeleopTemplate {

    public void runOpMode() throws InterruptedException {

        this.initAll();
        // this instantiation assumes that the color sensor's name is color_sensor
        intake = new ContinuousIntake(hardwareMap, "intake_motor", "bucketServo", "color_sensor", true);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("red:", intake.getColor("red"));
            telemetry.addData("green:", intake.getColor("green"));
            telemetry.addData("blue:", intake.getColor("blue"));
            telemetry.addData("argb", intake.getColor("argb"));
            telemetry.addData("alpha", intake.getColor("alpha"));
            telemetry.addData("distance:", intake.getCloseDistance());
            intake.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);


            telemetry.update();
        }
    }
}

