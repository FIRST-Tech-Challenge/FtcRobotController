package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.LiftArm;

@TeleOp(name="Manual Arm Control", group="Linear Opmode")
public class ManualArm extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // create instance of lift arm class
        LiftArm arm = new LiftArm(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.dpad_down) {
                arm.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.slide.setPower(-1);
                arm.setHandPosition(LiftArm.HandPosition.IN);
            } else if (gamepad1.dpad_up) {
                arm.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.slide.setPower(.5);
                arm.setHandPosition(LiftArm.HandPosition.IN);
            } else {
                arm.slide.setPower(0);
                arm.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (gamepad1.right_trigger > .75) {
                arm.hand.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.hand.setPower(-.35);
            } else if (gamepad1.left_trigger > .75) {
                arm.hand.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.hand.setPower(.35);
            } else {
                arm.hand.setPower(0);
                arm.hand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }
}


