package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class GripperTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor Shoulder_Motor = hardwareMap.dcMotor.get("Shoulder_Motor");
        Shoulder_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Servo Gripper = hardwareMap.servo.get("Gripper_Servo");

//        Servo servoTest = hardwareMap.get(Servo.class, "launch_servo");


        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            double modifier = 0.5;

            Shoulder_Motor.setPower((gamepad2.left_stick_x - gamepad2.left_stick_y - gamepad2.right_stick_x) * modifier);



            // check to see if we need to move the servo.
            if (gamepad2.b) {
                // move to 0 degrees.
                Gripper.setPosition(0);
            } else if (gamepad2.a) {
                // move to 90 degrees.
                Gripper.setPosition(0.4);
            }
        telemetry.addData("Servo Position", Gripper.getPosition());
//        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Status", "Running");
        telemetry.update();
        }

    }
}