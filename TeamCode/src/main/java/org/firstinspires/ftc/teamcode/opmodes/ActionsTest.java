package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.complex.ComplexActions;

@TeleOp(name = "ActionsTest")
public class ActionsTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            DcMotor leftElevator  = hardwareMap.dcMotor.get("armLeft");
            DcMotor rightElevator = hardwareMap.dcMotor.get("armRight");
            DcMotor frontArm      = hardwareMap.dcMotor.get("frontArm");

            Servo rotator         = hardwareMap.servo.get("rotator");
            Servo clawServo       = hardwareMap.servo.get("claw");

            Servo leftArm         = hardwareMap.servo.get("leftArm");
            Servo rightArm        = hardwareMap.servo.get("rightArm");
            Servo leftSlide       = hardwareMap.servo.get("slideServoLeft");
            Servo rightSlide      = hardwareMap.servo.get("slideServoRight");

            while (opModeIsActive()) {
                // OpMode loop

                ComplexActions comp = new ComplexActions(hardwareMap);
                if (gamepad1.a) {
                    rotator.setPosition(0.85);
                    sleep(1250);
                    Actions.runBlocking(comp.grabCubeFromTray());
                }

                /// Telemetry ///

            }
        }
    }
}
