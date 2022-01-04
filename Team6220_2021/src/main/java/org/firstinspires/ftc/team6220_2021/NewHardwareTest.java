//  This is a class for TeleOp tank drive.
//  You can install this program to the rev control hub without any edits.
//  Using the logitech controller, you can move the robot in tank drive.

package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "New Hardware AutonomousTest", group = "TeleOp")
public class NewHardwareTest extends MasterOpMode{

    int tickvalue = -70;
    double x = 0.7;

    //for run to position or manual control
    boolean toPosition = true;

    //Other Devices
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        Initialize();

        double position = 0.0;
        double servoArmPostion = 0.0;

        //Set position of servos
        servoGrabber.setPosition(0.34);
        servoArm.setPosition(0.3);

        //Declare variables
        int addingticks = 0;
        boolean isPressed = false;
        double motorPower = 0.9;
        double increase = 1;
        double speed = 1;
        int targetPostion = 0;

        waitForStart();

        //Set power of motors
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                targetPostion = 2800;
            } else if (gamepad1.right_bumper) {
                targetPostion = 0;
            }
            if (gamepad1.left_trigger > 0.9) {
                servoArmPostion += 0.001;
            } else if (gamepad1.right_trigger > 0.9) {
                servoArmPostion -= 0.001;
            }
            servoArm.setPosition(servoArmPostion);
            motorBelt.setPower(gamepad1.right_stick_x);

            if (gamepad1.left_stick_y > 0.5) {
                position += 0.001;
            } else if (gamepad1.left_stick_y < -0.5) {
                position -= 0.001;
            }

            if (position > 1) {
                position = 1;
            } else if (position < 0) {
                position = 0;
            }

            servoGrabber.setPosition(position);
            telemetry.addData("ServoPosition", position);
            telemetry.addData("ServoArmPosition", servoArmPostion);
            telemetry.addData("beltposition", motorBelt.getCurrentPosition());
            telemetry.update();
            motorLeftDuck.setPower(gamepad1.right_stick_y);
        }
    }
}