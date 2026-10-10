package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "FlywheelTestBiobuzz")
public class FlywheelTestBiobuzz extends LinearOpMode {
    private DcMotorEx flywheel = null;
    private DcMotor bottomCollection = null;
    private DcMotor topCollection = null;

    private double desiredFlywheelVelocity;

    public void processGamepadInput() {
        if (gamepad1.x) {
            bottomCollection.setPower(-1);
            topCollection.setPower(-1);
        } else if (gamepad1.left_bumper) {
            bottomCollection.setPower(1);
            topCollection.setPower(.8);
        } else {
            bottomCollection.setPower(0);
            topCollection.setPower(0);
        }

        if (gamepad1.right_bumper) {
            desiredFlywheelVelocity = -1500;
        } else if (gamepad1.dpad_up) {
            telemetry.addData("GamepadEvent", "dpadUp");
            desiredFlywheelVelocity -= 50;
            sleep(500);
        } else if (gamepad1.dpad_down) {
            telemetry.addData("GamepadEvent", "dpadDown");
            desiredFlywheelVelocity += 50;
            sleep(500);
        } else if (gamepad1.b) {
            desiredFlywheelVelocity = 0;
        } else {
            telemetry.addData("GamepadEvent", "NothingPressed");
        }
        flywheel.setVelocity(desiredFlywheelVelocity);

        telemetry.addData("DesiredVelocity", desiredFlywheelVelocity);
        telemetry.addData("ActualVelocity", flywheel.getVelocity());
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "Josh");
        bottomCollection = hardwareMap.get(DcMotor.class, "BottomCollection");
        topCollection = hardwareMap.get(DcMotor.class, "TopCollection");
        waitForStart();
        while (opModeIsActive()) {
            processGamepadInput();
        }
    }
}