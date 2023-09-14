package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.roadrunner.util.MathUtil;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivePose.DrivePose;
import org.firstinspires.ftc.teamcode.subsystem.MyCamera;

public class AlignAprilTag extends CommandBase {
    private Gamepad gamepad;
    private DrivePose drive;
    private MyCamera myCamera;
    private double[] tag = new double[4];
    private double translation, strafe, rotation;
    private double id, translationVal, strafeVal, rotationVal;
    private double translationError, strafeError, rotationError;
    private boolean hasTarget, isHasTarget, tolerance;
    private int no;
    private Telemetry telemetry;

    public AlignAprilTag(Telemetry telemetry, Gamepad gamepad, DrivePose drive, MyCamera myCamera, double id, double strafe, double translation, double rotation) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.drive = drive;
        this.myCamera = myCamera;
        this.id = id;
        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        addRequirements(drive, myCamera);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        isHasTarget = false;
        for (int i = 0; i < myCamera.getAprilTagData().length; i++) {
            if (myCamera.getAprilTagData()[i][0] == id) {
                tag = myCamera.getAprilTagData()[i];
                isHasTarget = true;
                break;
            }
        }
        if (isHasTarget) {
            no = 0;
            //move forward and backward
            if (tag[1] - strafe < 30.0) {
                strafeVal = MathUtils.clamp((tag[1] - strafe) * 0.1, -0.4, 0.4);
                strafeVal += Math.signum(strafeVal) * 0.008;
            } else {
                strafeVal = 0.0;
            }
            //Move left and right
            double dist = Math.abs(tag[1] - strafe);
            double kp2 = dist > 4.0 ? dist / 3.0 * dist / 3.0 : 1.0;
            translationVal = MathUtils.clamp((tag[2] - translation) * 0.6 * kp2, -0.32, 0.32);
            translationVal += Math.signum(translationVal) * 0.004;
            //move rotate
            double kp3 = dist > 4.0 ? 1.3 : 1.0;
            rotationVal = MathUtils.clamp((rotation - tag[3]) * 0.2 * kp3, -0.13, 0.13);
            rotationVal += Math.signum(rotationVal) * 0.006;

            drive.driveJoy(strafeVal, //forward and backward
                    translationVal, //left and right
                    rotationVal); //rotate left and right
            tolerance = (Math.abs(strafeVal) < 0.03) && (Math.abs(translationVal) < 0.055) && (Math.abs(rotationVal) < 0.02);

            telemetry.addData("Error1", strafeVal);
            telemetry.addData("Error2", translationVal);
            telemetry.addData("Error3", rotationVal);
        } else {
            drive.driveJoy(0.0, 0.0, 0.0);
            no++;
        }
//        telemetry.addData("info0", tag[0]);
//        telemetry.addData("info1", tag[1]);
//        telemetry.addData("info2", tag[2]);
//        telemetry.addData("info3", tag[3]);
        telemetry.update();
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveJoy(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return (!isHasTarget && no > 20) || gamepad.left_stick_button || tolerance;
    }
}
