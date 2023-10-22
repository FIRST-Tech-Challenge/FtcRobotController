package org.firstinspires.ftc.teamcode;

import android.icu.util.MeasureUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DriverOperationMode")
public class DriverOp extends RobotOpMode {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void robotloop() {
        gamePadMoveRobot();
        int direction = 0;
        if (gamepad1.dpad_up) {
            int position = positionFromAngle(-110, AngleUnit.DEGREES);
            armMotor.setTargetPosition(position);
            armMotor.setPower(1);
        }
        else if (gamepad1.dpad_down) {
            int position = positionFromAngle(0, AngleUnit.DEGREES);
            armMotor.setTargetPosition(position);
            armMotor.setPower(1);
        }
        if(!armMotor.isBusy()) {
            armMotor.setPower(0);
        }
    }

    public int positionFromAngle(double angle, AngleUnit angleUnit) {
        double ticksPerRevolution = armMotor.getMotorType().getTicksPerRev();
        double scale = angleUnit.toDegrees(angle)/360;
        return (int) (ticksPerRevolution*scale);
    }

}
