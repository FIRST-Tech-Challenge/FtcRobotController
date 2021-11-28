package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.DriveMotor;

/** Tests the velocity PID and rotation reporting of the drive motor class.
 * @author TheConverseEngineer
 */
@TeleOp(name = "Drive Motor Test", group = "Tests")
public class DriveMotorTest extends OpMode {

    private DriveMotor driveMotor;
    private boolean xAlreadyPressed;

    @Override
    public void init() {
        driveMotor = new DriveMotor(hardwareMap, "rightFrontDrive", DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Testing:", "rightFrontDrive");
        telemetry.update();
        xAlreadyPressed = false;
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            telemetry.addData("Velocity", "1 rps");
            driveMotor.setVelocity(1);
        } else if (gamepad1.b) {
            telemetry.addData("Velocity", "4 rps");
            driveMotor.setVelocity(4);
        }

        if (gamepad1.x) {
            if (!xAlreadyPressed) {
                xAlreadyPressed = true;
                telemetry.addData("Total rotations", driveMotor.getDPos());
            }
        } else {
            xAlreadyPressed = false;
        }
        telemetry.update();
    }
}
