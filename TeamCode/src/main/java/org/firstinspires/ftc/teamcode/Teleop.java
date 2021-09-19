package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.core.FtcGamePad;
import com.bravenatorsrobotics.drive.MecanumDrive;
import com.bravenatorsrobotics.operation.TeleopMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Teleop")
public class Teleop extends TeleopMode<MecanumDrive> {

    private DcMotorEx lift;

    // Create TeleopMode with specified specifications
    public Teleop() { super(new Specifications()); }

    @Override
    public void OnInitialize() {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        switch(button) {
            case FtcGamePad.GAMEPAD_DPAD_UP:
                if(pressed) {
                    lift.setPower(1);
                } else {
                    lift.setPower(0);
                }
                break;
            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                if(pressed) {
                    lift.setPower(-1);
                } else {
                    lift.setPower(0);
                }
                break;
        }
    }

    @Override
    protected void OnOperatorGamePadChange(FtcGamePad gamePad, int button, boolean pressed) {

    }

}
