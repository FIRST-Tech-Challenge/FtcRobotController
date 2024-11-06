
package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
public class TeamMotorTestAV extends LinearOpMode {
    private DcMotor armMotor;

    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        double tgtPower = 0;

        telemetry.addData("Hello", ", Team RoboActive💀🔥🙊🗣🗿");
        telemetry.addData("Status", "Initialized🗿🔥");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double position = 0.0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running🏃‍♂️💨");
            telemetry.addData("Status", "Game Started...🎮🔥🔥");
            telemetry.update();

            if (gamepad1.y) {
                armMotor.setDirection(DcMotor.Direction.FORWARD);
            } else if (gamepad1.a) {
                armMotor.setDirection(DcMotor.Direction.REVERSE);
            } else if (this.gamepad1.left_stick_y == 1) {
                armMotor.setTargetPosition(200);
                armMotor.setPower(-0.1);
            } else if (this.gamepad1.left_stick_y == -1) {
                armMotor.setTargetPosition(1);
                armMotor.setPower(0.1);
            } else if (gamepad1.left_stick_x == 1) {
                armMotor.setPower(0.1);
            } else if (gamepad1.b) {
                armMotor.setPower(0);
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else if (gamepad1.x || gamepad1.right_trigger == 1.0) {
                double power = armMotor.getPower();
                armMotor.setPower(power + 0.1);
            } else if (gamepad1.x || gamepad1.right_bumper) {
                double power = armMotor.getPower();
                armMotor.setPower(power - 0.1);
            }


            waitForStart();
            telemetry.addData("left wheel motor position", armMotor.getTargetPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
































































































































































































































































































































































