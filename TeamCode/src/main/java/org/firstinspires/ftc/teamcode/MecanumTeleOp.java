package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp Program", group="TeleOp")
public class MecanumTeleOp extends OpMode {

    Robot robot = new Robot();


    //Code to run ONCE after the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
    }


    //Code to run REPEATEDLY after the driver hits INIT
    @Override
    public void init_loop() {
        telemetry.update();

    }

    //Code to run ONCE after the driver hits PLAY
    @Override
    public void start() {


    }

    //Code to run REPEATEDLY after the driver hits PLAY
    @Override
    public void loop() {
        double[] driveVelocities =
                robot.driveTrain.drive(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x);

        robot.driveTrain.setDriveVelocities(driveVelocities);



        if (gamepad1.dpad_up) {
            robot.linear_L.linear_motion_left.setPower(1);
            robot.linear_R.linear_motion_right.setPower(1);
        } else if (gamepad1.dpad_down) {
            robot.linear_L.linear_motion_left.setPower(-1);
            robot.linear_R.linear_motion_right.setPower(-1);

        } else {
            robot.linear_L.linear_motion_left.setPower(0.005);
            robot.linear_R.linear_motion_right.setPower(0.005);
        }
    }


}
