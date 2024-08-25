package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "AvyuktTeleOp")
public class TeleOpMode extends OpMode {
ChassisAvyukt theAmazingChassis;
double x;
double y;
double rx;
LiftAvyukt liftAvyukt;
    @Override
    public void init() {
        theAmazingChassis = new ChassisAvyukt(hardwareMap);
        liftAvyukt = new LiftAvyukt(hardwareMap);
    }

    @Override
    public void loop() {
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x * 1.1;
        rx = gamepad1.right_stick_x;
        telemetry.addData("Lift Encoder", LiftAvyukt.getEncoderValue());
        telemetry.addData("Initialization", value: "Complete");
        telemetry.addData("left stick y", gamepad1.left_stick_y);
        telemetry.addData("right stick y", gamepad1.right_stick_y);
        telemetry.addData("left stick x", gamepad1.left_stick_x);
        telemetry.addData("right stick x", gamepad1.right_stick_x);

        theAmazingChassis.drive(x,y,rx);
    }
}
