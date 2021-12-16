package ca.webber.ftc.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import ca.webber.ftc.helpers.AutoDriveController;
import ca.webber.ftc.helpers.TeleDriveController;

@Autonomous(name="Autonomous Mode 1")
@Disabled
public class Auto_01 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private AutoDriveController autoDriveController;

    @Override
    public void runOpMode () {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // instantiates the driveController
        autoDriveController = new AutoDriveController(
                hardwareMap.get(DcMotor.class, "leftFront"),
                hardwareMap.get(DcMotor.class, "rightFront"),
                hardwareMap.get(DcMotor.class, "leftBack"),
                hardwareMap.get(DcMotor.class, "rightBack")
        );

        boolean hasRan = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // operates the driveController
            if (!autoDriveController.isBusy() && !hasRan) {
                autoDriveController.setMovement(1, 1);
                hasRan = true;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
