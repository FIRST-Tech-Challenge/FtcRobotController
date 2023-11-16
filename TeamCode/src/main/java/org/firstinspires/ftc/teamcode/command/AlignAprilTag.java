package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DrivePose;
import org.firstinspires.ftc.teamcode.subsystem.MyCamera;
import org.firstinspires.ftc.teamcode.util.RollingAverage;

public class AlignAprilTag extends CommandBase {
    private Gamepad gamepad;
    private DrivePose drive;
    private MyCamera myCamera;
    private double[] tag = new double[4];
    private double translation, strafe, rotation;
    private double translationVal, strafeVal, rotationVal, translationFriction, strafeFriction, rotationFriction;
    private double lastX, lastY;
    private boolean  isHasTarget, tolerance, standStill;
    private int id, invalidNo, runNo;
    private RollingAverage rollAverX = new RollingAverage(5);
    private RollingAverage rollAverY = new RollingAverage(5);
    private Telemetry telemetry;

    public AlignAprilTag(Telemetry telemetry, Gamepad gamepad, DrivePose drive, MyCamera myCamera, int id, double strafe, double translation, double rotation) {
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
        runNo = 0;
        standStill = false;
        isHasTarget = true;
        tolerance = false;
    }

    @Override
    public void execute() {
        tag = myCamera.getAprilTagIDData(id);
        if (tag[0] == id) {
            isHasTarget = true;
        } else {
            isHasTarget = false;
        }
        if (isHasTarget) {
            invalidNo = 0;
            //move forward and backward
            if (tag[1] - strafe < 30.0) {
                strafeVal = MathUtils.clamp((tag[1] - strafe) * 0.1, -0.4, 0.4);
                strafeFriction = Math.signum(strafeVal) * 0.008;
            } else {
                strafeVal = 0.0;
            }
            //Move left and right
            double dist = Math.abs(tag[1] - strafe);
            double kp2 = dist > 4.0 ? dist / 3.0 * dist / 3.0 : 1.0;
            translationVal = MathUtils.clamp((tag[2] - translation) * 0.35 * kp2, -0.32, 0.32);
            translationFriction = Math.signum(translationVal) * 0.006;
            //move rotate
            double kp3 = dist > 4.0 ? 1.3 : 1.0;
            rotationVal = MathUtils.clamp((rotation - tag[3]) * 0.2 * kp3, -0.13, 0.13);
            rotationFriction = Math.signum(rotationVal) * 0.01;
            //Normal drive use rear camera track Tag
            drive.driveJoy(-(strafeVal * 0.5 + strafeFriction), //forward and backward
                           -(translationVal * 0.5 + translationFriction), //left and right
                            (rotationVal * 0.5 + rotationFriction)); //rotate left and right
            //Rotate drive use rear camera track Tag
//            drive.driveJoy(
//                    -(translationVal * 0.5 + translationFriction), //left and right
//                    (strafeVal * 0.5 + strafeFriction), //forward and backward
//                    (rotationVal * 0.5 + rotationFriction)); //rotate left and right
            tolerance = (Math.abs(strafeVal) < 0.02) && (Math.abs(translationVal) < 0.035) && (Math.abs(rotationVal) < 0.025);

            telemetry.addData("Error1", strafeVal);
            telemetry.addData("Error2", translationVal);
            telemetry.addData("Error3", rotationVal);
            rollAverX.add(drive.getMyPose()[0]);
            rollAverY.add(drive.getMyPose()[1]);
//            telemetry.addData("ErrorX", Math.abs(rollAverX.getAverage() - lastX));
//            telemetry.addData("ErrorY", Math.abs(rollAverY.getAverage() - lastY));
            runNo++;
            if (runNo%10==0) {
                runNo = 0;
                if ((Math.abs(strafeVal) < 0.09) && (Math.abs(translationVal) < 0.15) && (Math.abs(rotationVal) < 0.06)) {
                    if (Math.abs(rollAverX.getAverage() - lastX)<0.03&&Math.abs(rollAverY.getAverage() - lastY)<0.03) {
                         standStill= true;
                    }
                }
                lastX = rollAverX.getAverage();
                lastY = rollAverY.getAverage();
            }
        } else {
            drive.driveJoy(0.0, 0.0, 0.0);
            invalidNo++;
        }
//        telemetry.addData("info0", tag[0]);
//        telemetry.addData("info1", tag[1]);
//        telemetry.addData("info2", tag[2]);
//        telemetry.addData("info3", tag[3]);
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveJoy(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return (!isHasTarget && invalidNo > 20) || gamepad.left_stick_button || tolerance || standStill;
    }
}
