package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "Testing")
public class GripperCalibration extends LinearOpMode {

    String expectedState;
    String lastPressed = "";

    double adjustment = 0.001;
    double debounceDelay = 0.01;
    double debounceTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor Shoulder_Motor = hardwareMap.dcMotor.get("Shoulder_Motor");
        Shoulder_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Servo Gripper = hardwareMap.servo.get("Gripper_Servo");

        Robot2024 robot2024 = new Robot2024(Gripper, Shoulder_Motor);

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            telemetry.addLine("Status: Running Gripper Position Tests");
            telemetry.addLine();

            telemetry.addLine("You pressed '" + this.lastPressed + "' last");
            telemetry.addLine("The gripper should be '" + this.expectedState + "'");
            telemetry.addData("Servo Position", Gripper.getPosition());
            telemetry.addLine();

            telemetry.addData("Servo Close Position", robot2024.gripper_close_position);
            telemetry.addData("Servo Open Position", robot2024.gripper_open_position);
            telemetry.addData("Servo Adjustment Size", adjustment);
            telemetry.addLine();

            telemetry.addLine("Press 'a' to Open the Grippers");
            telemetry.addLine("Press 'b' to Close the Grippers");
            telemetry.addLine("Press '^' or 'v' to change the open position");
            telemetry.addLine("Press '<' or '>' to change the close position");
            telemetry.addLine("Press '<' or '>' to change the close position");
            telemetry.addLine("Press 'LB' or 'RB' to change the close position");
            telemetry.addLine();

            telemetry.addLine("If you've found the correct values, write them down");
            telemetry.addLine("Then update the 'Robot2024.java' file");
            telemetry.addLine();

            if (gamepad2.b) {
                robot2024.closeGripper();
                this.expectedState = "Closed";
                this.lastPressed = "b";
            } else if (gamepad2.a) {
                robot2024.openGripper();
                this.expectedState = "Opened";
                this.lastPressed = "a";
            }

            telemetry.addData("Time", this.time);
            if (this.time - this.debounceTime > this.debounceDelay) {
                if (gamepad2.dpad_left) {
                    robot2024.gripper_close_position = robot2024.gripper_close_position - adjustment;
                } else if (gamepad2.dpad_right) {
                    robot2024.gripper_close_position = robot2024.gripper_close_position + adjustment;
                } else if (gamepad2.dpad_down) {
                    robot2024.gripper_open_position = robot2024.gripper_open_position - adjustment;
                } else if (gamepad2.dpad_up) {
                    robot2024.gripper_open_position = robot2024.gripper_open_position + adjustment;
                } else if (gamepad2.left_bumper) {
                    adjustment -= 0.001;
                } else if (gamepad2.right_bumper) {
                    adjustment += 0.001;
                }
                this.debounceTime = this.time;
            }

        telemetry.update();
        }

    }
}