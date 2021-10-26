package org.firstinspires.ftc.teamcode.other.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.GamepadExtended;

public class TankRobot extends GamepadExtended {

    public DrivetrainManager4WD drivetrainManager4WD;
    public Tank tank;
    public DcMotor spinner;

    public TankRobot(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2, telemetry);
        tank = new Tank(
                telemetry,
                new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.RIGHT_DRIVE_1), DcMotorSimple.Direction.FORWARD),
                new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.RIGHT_DRIVE_2), DcMotorSimple.Direction.FORWARD),
                new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.LEFT_DRIVE_1), DcMotorSimple.Direction.FORWARD),
                new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.LEFT_DRIVE_2), DcMotorSimple.Direction.FORWARD)
        );
    }

    @Override
    public void main() {
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.left_stick_x;
        double left    = Range.clip((drive + turn)*100, -100.0, 100.0) ;
        double right   = Range.clip((drive - turn)*100, -100.0, 100.0) ;

        tank.driveWithoutEncoder((int) -right, (int) -left);

        /*
        if ((gamepad2.left_stick_y >= 0.25 | gamepad2.left_stick_y <= -0.25) && priority.f2(false)) {
            spinner.setPower(gamepad2.left_stick_y);
        }
        else if (gamepad1.right_trigger >= 0.25) { spinner.setPower(gamepad1.left_stick_y); }
        */
    }
}
