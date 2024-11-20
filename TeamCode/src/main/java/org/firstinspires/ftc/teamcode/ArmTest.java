package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArmTest extends LinearOpMode {



    @Override
    public void runOpMode() {
        Servo intake = hardwareMap.get(Servo.class, "Intake");

        DcMotor arm = hardwareMap.get(DcMotor.class, "Arm");
        DcMotor wrist = hardwareMap.get(DcMotor.class, "Wrist");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        intake.setDirection(Servo.Direction.FORWARD);
        intake.getController().pwmEnable();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

                arm.setPower(gamepad2.left_stick_y*.5);
            wrist.setPower(gamepad2.right_stick_y*.5);
            telemetry.addData("Intake", intake.getPosition());
            telemetry.addData("New Intake Position", (intake.getPosition() + .01f) % 1.0f);
            if(gamepad2.right_bumper) {
                intake.setPosition((intake.getPosition()+.01)% 1.0);
                sleep(50);
                idle();
            }




        }
    }


}
