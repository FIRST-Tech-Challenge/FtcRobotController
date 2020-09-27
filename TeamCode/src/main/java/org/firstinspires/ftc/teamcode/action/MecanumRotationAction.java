package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;
import org.firstinspires.ftc.teamcode.util.EncoderDrive;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

/**
 * Created by djfigs1 on 10/1/17.
 */

public class MecanumRotationAction implements Action {

    private int degrees;
    private float speed;
    EncoderDrive encoderDrive;

    public MecanumRotationAction(int degrees, float speed) {
        this.degrees = degrees;
        this.speed = speed;
    }

    @Override
    public void init(RobotHardware hardware) {
        int distance = (int) (hardware.COUNTS_PER_DEGREE * degrees);
        encoderDrive = new EncoderDrive(hardware);
        if (distance < 0) {
            distance = -distance;
            encoderDrive.setCountsToDrive(OmniDrive.Direction.ROTATE_LEFT, distance, speed, 1500);
        } else {
            encoderDrive.setCountsToDrive(OmniDrive.Direction.ROTATE_RIGHT, distance, speed, 1500);
        }
    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        encoderDrive.run(hardware);
        return !encoderDrive.isBusy;
    }

    @Override
    public Double progress() {
        return null;
    }

    @Override
    public String progressString() {
        return null;
    }
}
