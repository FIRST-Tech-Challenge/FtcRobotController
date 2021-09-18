package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.core.FtcGamePad;
import com.bravenatorsrobotics.drive.MecanumDrive;
import com.bravenatorsrobotics.operation.TeleopMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
        // 23,800
    }

    @Override
    public void OnStop() {
        robot.drive.Stop();
    }

    @Override
    protected void OnDriverGamePadChange(FtcGamePad gamePad, int button, boolean pressed) {

    }

    @Override
    protected void OnOperatorGamePadChange(FtcGamePad gamePad, int button, boolean pressed) {

    }

}
