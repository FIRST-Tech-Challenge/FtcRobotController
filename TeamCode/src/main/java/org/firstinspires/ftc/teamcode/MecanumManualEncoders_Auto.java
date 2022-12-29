package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="MecanumManualEncoders", group="Linear Opmode")
public class MecanumManualEncoders_Auto extends LinearOpMode {

    // Declare hardware:
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;

    @Override
    public void runOpMode() {

        // Configure hardware map:
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");

        // Set motor directions:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders:
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Shows driver that encoders have successfully been reset:
        telemetry.addData("Starting Position", "%7d :%7d :%7d :%7d", leftFrontMotor.getCurrentPosition(),
                rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                rightBackMotor.getCurrentPosition());
        telemetry.update();

        // Waits for driver to press play:
        waitForStart();

        // Clears telemetry to allow for live encoder position telemetry:
        telemetry.clear();

        // Gives power to motors until encoder position 100 has been reached:
        // opModeIsActive() must be required in all while loops.
        while (leftFrontMotor.getCurrentPosition() < 100 && opModeIsActive()) {

            // Sets all motors to 0.6 power:
            leftFrontMotor.setPower(0.6);
            rightFrontMotor.setPower(0.6);
            leftBackMotor.setPower(0.6);
            rightBackMotor.setPower(0.6);

            // Shows live encoder count positions:
            telemetry.addData("Encoder Position", "%7d :%7d :%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Sets all powers to zero once encoder count 100 has been reached:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        // Clears telemetry to allow for final encoder position telemetry:
        telemetry.clear();

        // Sends snapshot of final positions:
        telemetry.addData("Final Position", "%7d :%7d :%7d :%7d", leftFrontMotor.getCurrentPosition(),
                rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                rightBackMotor.getCurrentPosition());
        telemetry.update();

        // Pause for 5 seconds for final snapshot of the final positions:
        sleep(5000);
    }


}
