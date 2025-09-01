package org.firstinspires.ftc.team5898.archive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team5898.RobotHardware;

@TeleOp(name="Doomsday", group="Competition")
public class DoomsDay extends LinearOpMode {
    private RobotHardware robot;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        waitForStart();
        robot.slideLeft.setTargetPosition(robot.slideLeft.getCurrentPosition());
        robot.slideRight.setTargetPosition(robot.slideRight.getCurrentPosition());
        robot.slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive()) {
            /** manual control
            if (gamepad1.dpad_up) {
                robot.syncLift(0.8); // Raise lift with PID correction
            } else if (gamepad1.dpad_down) {
                robot.syncLift(-0.5); // Lower lift with PID correction
            } else {
                robot.syncLift(0); // Stop lift
            }**/

            if (gamepad2.dpad_down){
                robot.slideLeft.setTargetPosition(robot.slideLeft.getCurrentPosition() - 20);
                robot.slideRight.setTargetPosition(robot.slideRight.getCurrentPosition() - 20);
                robot.slideLeft.setPower(.5);
                robot.slideRight.setPower(.5);
            }
            else {
                robot.slideLeft.setPower(0);
                robot.slideRight.setPower(0);
            }

        }
    }
}
