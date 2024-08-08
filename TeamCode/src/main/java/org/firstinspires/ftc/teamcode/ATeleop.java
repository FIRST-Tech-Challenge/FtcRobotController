package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ATeleop extends LinearOpMode {
    public RobotDrive bot;
    public DualPad gpad;

    @Override
    public void runOpMode() {
        bot = new RobotDrive();
        Bot nondrivebot = new Bot();
        bot.init(hardwareMap);

        gpad = new DualPad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            gpad.update(gamepad1, gamepad2);
            double jx = -gamepad1.left_stick_y - gamepad1.right_stick_y;
            double jy = -gamepad1.left_stick_x;
            double jw = -gamepad1.right_stick_x;

            if (gpad.start) {
                if (gpad.dpad_up) bot.setHeading(0);
                if (gpad.dpad_right) bot.setHeading(270);
                if (gpad.dpad_down) bot.setHeading(180);
                if (gpad.dpad_left) bot.setHeading(90);
            }

            if (gpad.right_bumper) {
                nondrivebot.intake(true);
            }

            if (gpad.right_trigger > 0.0) {
                nondrivebot.intake(false);//intake reversed
            }
            if (gpad.left_trigger > 0.0){
                jx = nondrivebot.setspeed(gpad.left_trigger, jx);
                jy = nondrivebot.setspeed(gpad.left_trigger, jy);
                jw = nondrivebot.setspeed(gpad.left_trigger, jw);

            }
            bot.driveXYW( jx, jy, jw);

            telemetry.addData("Status", "Running");
            telemetry.addData("heading", bot.getHeading());
            telemetry.addData("IMU heading", bot.getIMUHeading());
            telemetry.update();
        }
    }
}
