package org.firstinspires.ftc.teamcode.src.DrivePrograms.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.Utills.TeleopTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;

@TeleOp(name = "LS Test")
public class LinearSlideTest extends TeleopTemplate {
    LinearSlide linearSlide;

    public void runOpMode() {
        this.initAll();
        RobotVoltageSensor s = new RobotVoltageSensor(hardwareMap);
        linearSlide = new LinearSlide(hardwareMap, "slide_motor", s, this::opModeIsActive, this::isStopRequested);
        Thread t = new Thread(linearSlide);
        linearSlide.setTargetLevel(LinearSlide.HeightLevels.Down);


        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();
        t.start();
        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setPowerFromGamepad(gamepad1);

            linearSlide.setTargetLevel(LinearSlide.HeightLevels.BottomLevel);
            //linearSlide.updatePower();

            //intake.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);
            telemetry.addData("LS Height: ", linearSlide.getEncoderCount());
            telemetry.update();

        }
    }
}