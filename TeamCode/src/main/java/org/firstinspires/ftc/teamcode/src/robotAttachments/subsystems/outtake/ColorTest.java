package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.outtake;

import static org.firstinspires.ftc.teamcode.src.utills.enums.RGBCameraColors.Blue;
import static org.firstinspires.ftc.teamcode.src.utills.enums.RGBCameraColors.Green;
import static org.firstinspires.ftc.teamcode.src.utills.enums.RGBCameraColors.Red;

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
    public void opModeMain() throws InterruptedException {
        this.initOuttake();
        this.initIntake();

        // this instantiation assumes that the color sensor's name is color_sensor
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            intake.gamepadControl(gamepad1, gamepad2);

            outtake.gamepadControl(gamepad1, gamepad2);

            telemetry.addData("red:", outtake.getColor(Red));
            telemetry.addData("green:", outtake.getColor(Green));
            telemetry.addData("blue:", outtake.getColor(Blue));
            telemetry.addData("identity:", FreightFrenzyGameObject.identify(outtake.getRGB()));
            telemetry.addData("Color", FreightFrenzyGameObject.getLEDColorFromItem(FreightFrenzyGameObject.identify(outtake.getRGB())));

            telemetry.update();
        }
    }
}

