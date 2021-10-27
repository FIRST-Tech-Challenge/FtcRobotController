package org.firstinspires.ftc.teamcode.other.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.GamepadExtended;

import java.util.ArrayList;
import java.util.List;

public class TwoWDRobotWithSpinnerAndLift extends GamepadExtended {

    public DrivetrainManager4WD drivetrainManager4WD;
    public DcMotor spinner;
    public DcMotor lift;

    public TwoWDRobotWithSpinnerAndLift(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2, telemetry);

        List<String> motorNames = new ArrayList<String>();

        motorNames.add(hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_1));
        motorNames.add(hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_1));

        this.drivetrainManager4WD = new DrivetrainManager4WD(motorNames, hardwareMap);

        spinner = hardwareMap.dcMotor.get(hardwareMap.appContext.getString(R.string.HW_SPINNER));
        lift = hardwareMap.dcMotor.get(hardwareMap.appContext.getString(R.string.HW_LIFT));
    }

    @Override
    public void main() {
        drivetrainManager4WD.EvalGamepad(gamepad1.left_stick_x, gamepad1.left_stick_y);

        if ((gamepad2.left_stick_y >= 0.25 | gamepad2.left_stick_y <= -0.25) && priority.f2(false)) {
            spinner.setPower(gamepad2.left_stick_y);
        }
        else if (gamepad1.right_trigger >= 0.25) { spinner.setPower(gamepad1.left_stick_y); }



        if ((gamepad2.left_stick_x >= 0.25 | gamepad2.left_stick_x <= -0.25) && priority.f3(false)) {
            lift.setPower(gamepad2.left_stick_x);
        }
        else if (gamepad1.left_trigger >= 0.25) { lift.setPower(gamepad1.left_trigger); }
    }
}
