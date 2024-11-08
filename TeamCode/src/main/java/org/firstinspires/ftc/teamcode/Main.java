package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.functions.OmniDrive;
import org.firstinspires.ftc.teamcode.functions.SlideFunctions;

@TeleOp(name="Main", group="Linear OpMode")
public class Main extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();

    
    @Override
    public void runOpMode() {

        // Initialize hardware and omniDriveHelper
        OmniDrive OmniFunction = new OmniDrive(hardwareMap);
        SlideFunctions Slides = new SlideFunctions(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Initialized","Status");
        telemetry.update();
        waitForStart();
        runtime.reset();


        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Call the omniFunction method and pass in the gamepad and telemetry
            OmniFunction.OmniUpdate(gamepad1,telemetry);
            Slides.ArmPosition(gamepad1, gamepad2, telemetry);
            Slides.SlidePosition(gamepad1, gamepad2, telemetry);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
