package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.drive.MecanumDrive;
import com.bravenatorsrobotics.operation.TeleopMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop")
public class Teleop extends TeleopMode<MecanumDrive> {

    // Create TeleopMode with specified specifications
    public Teleop() { super(new Specifications()); }

    @Override
    public void OnInitialize() {

    }

    @Override
    public void OnUpdate() {
        robot.drive.Drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    @Override
    public void OnStop() {
        robot.drive.Stop();
    }

}
