package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AnglePIDControl;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;

public class PacManTurnToPos {
private double heading     = 0;
private double goalHeading = 0;
private double turnSpeed   = 0;
private Localizer localizer;
private  MecanumDriveBase mecanumDriveBase;
private AnglePIDControl angleControl;
CompassSensor                compass;



    public PacManTurnToPos(Localizer localizer, MecanumDriveBase mecanumDriveBase) {
        this.localizer = localizer;
        this.mecanumDriveBase = mecanumDriveBase;
        this.angleControl = new AnglePIDControl(.01, 0, .003,360);
    }

    public void handlePacMan(Gamepad gamepad, Telemetry telemetry) {
        //makes heading easier for me.
        heading =  localizer.heading;
        if(gamepad.right_stick_y < 10 || gamepad.right_stick_x < 10 || gamepad.left_stick_y < 10 || gamepad.left_stick_x < 10) {
            if (gamepad.dpad_down) {
                angleControl.setTargetValue(180);
                mecanumDriveBase.driveMotors(0, angleControl.update(heading), 0, 1);
            }
            if (gamepad.dpad_up) {
                angleControl.setTargetValue(0);
                mecanumDriveBase.driveMotors(0, angleControl.update(heading), 0, 1);
            }
            if (gamepad.dpad_left) {
                angleControl.setTargetValue(270);
                mecanumDriveBase.driveMotors(0, angleControl.update(heading), 0, 1);
            }
            if (gamepad.dpad_right) {
                angleControl.setTargetValue(90);
                mecanumDriveBase.driveMotors(0, angleControl.update(heading), 0, 1);

            }
            telemetry.addData("targetValue", angleControl.targetValue);
            telemetry.addData("target - measurement", angleControl.targetValue);
            telemetry.addData("measurement Error", angleControl.measuredError(heading));
        }
    }
}
