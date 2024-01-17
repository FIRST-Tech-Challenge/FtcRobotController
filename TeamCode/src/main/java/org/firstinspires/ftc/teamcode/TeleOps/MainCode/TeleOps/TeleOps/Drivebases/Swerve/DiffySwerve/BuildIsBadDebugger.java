package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Drivebases.Swerve.DiffySwerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Build Sucks")
public class BuildIsBadDebugger extends LinearOpMode {

    public DcMotor[] motors;
    int activeMotorIndex = 0; // Initialize the active motor index

    @Override
    public void runOpMode() throws InterruptedException {
        motors = new DcMotor[] {
                hardwareMap.dcMotor.get("LT"),
                hardwareMap.dcMotor.get("LB"),
                hardwareMap.dcMotor.get("RT"),
                hardwareMap.dcMotor.get("RB"),
        };

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                // Switch between motors based on the button press
                switch (activeMotorIndex) {
                    case 0:
                        activeMotorIndex = 1;
                        break;
                    case 1:
                        activeMotorIndex = 2;
                        break;
                    case 2:
                        activeMotorIndex = 3;
                        break;
                    case 3:
                        activeMotorIndex = 0;
                        break;
                    default:
                        activeMotorIndex = 0; // Set a default value to handle unexpected cases
                        break;
                }
            }

            // Assume gamepad1.right_trigger is the trigger value
            double triggerValue = gamepad1.right_trigger;

            // Ensure the active motor index is within bounds
            activeMotorIndex = Range.clip(activeMotorIndex, 0, 3);

            // Set the power of the active motor
            motors[activeMotorIndex].setPower(triggerValue);

            // Telemetry updates
            telemetry.addData("Active Motor", "Motor " + activeMotorIndex);
            telemetry.addData("Trigger Value", triggerValue);
            telemetry.update();
        }
    }
}
