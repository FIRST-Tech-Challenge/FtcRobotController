package org.firstinspires.ftc.teamcode.action;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

public class DirectionalMoveAction implements Action {

    OmniDrive.Direction direction;
    double distance;
    float speed;

    final int ENCODER_THRESHOLD = 20;
    int SLOW_COUNTS;
    final float SLOW_SPEED = 0.1f;
    float SPEED_SLOPE;

    int FL_target = 0;
    int FR_target = 0;
    int BL_target = 0;
    int BR_target = 0;
    int pos;

    double prevAllAvg = 9999999;
    double prevLeftRightAvg = 9999999;
    double prevRightLeftAvg = 9999999;
    double slowDistance;

    public DirectionalMoveAction(OmniDrive.Direction direction, double distance, float speed, float slowDistance) {
        this.direction = direction;
        this.distance = distance;
        this.speed = speed;
        this.slowDistance = slowDistance;
    }

    @Override
    public void init(RobotHardware hardware) {
        hardware.omniDrive.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.omniDrive.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.omniDrive.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.omniDrive.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int FL_pos = hardware.omniDrive.frontLeft.getCurrentPosition();
        int FR_pos = hardware.omniDrive.frontRight.getCurrentPosition();
        int BL_pos = hardware.omniDrive.backLeft.getCurrentPosition();
        int BR_pos = hardware.omniDrive.backRight.getCurrentPosition();
        pos = ((int) (this.distance * hardware.omniDrive.getCountsPerInch()));

        SLOW_COUNTS = (int) (slowDistance * hardware.omniDrive.getCountsPerInch());
        SPEED_SLOPE = (speed-SLOW_SPEED)/(SLOW_COUNTS);

        switch (direction) {
            case FORWARD:
                FL_target = FL_pos + pos;
                FR_target = FR_pos + pos;
                BL_target = BL_pos + pos;
                BR_target = BR_pos + pos;
                break;
            case LEFT:
                FL_target = FL_pos - pos;
                FR_target = FR_pos + pos;
                BL_target = BL_pos + pos;
                BR_target = BR_pos - pos;
                break;
            case RIGHT:
                FL_target = FL_pos + pos;
                FR_target = FR_pos - pos;
                BL_target = BL_pos - pos;
                BR_target = BR_pos + pos;
                break;
            case BACKWARD:
                FL_target = FL_pos - pos;
                FR_target = FR_pos - pos;
                BL_target = BL_pos - pos;
                BR_target = BR_pos - pos;
                break;
            case FORWARD_LEFT:
                FR_target = FR_pos + pos;
                BL_target = BL_pos + pos;
                break;
            case FORWARD_RIGHT:
                FL_target = FL_pos + pos;
                BR_target = BR_pos + pos;
                break;
            case BACKWARD_LEFT:
                FL_target = FL_pos - pos;
                BR_target = BR_pos - pos;
                break;
            case BACKWARD_RIGHT:
                FR_target = FR_pos - pos;
                BL_target = BL_pos - pos;
                break;
            default:
                break;
        }

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        int FL_pos = hardware.omniDrive.frontLeft.getCurrentPosition();
        int FR_pos = hardware.omniDrive.frontRight.getCurrentPosition();
        int BL_pos = hardware.omniDrive.backLeft.getCurrentPosition();
        int BR_pos = hardware.omniDrive.backRight.getCurrentPosition();

        int FL_difference = FL_target - FL_pos;
        int FR_difference = FR_target - FR_pos;
        int BL_difference = BL_target - BL_pos;
        int BR_difference = BR_target - BR_pos;

        double allAvg = (Math.abs(FL_difference) + Math.abs(FR_difference) + Math.abs(BL_difference) + Math.abs(BR_difference)) / 4.0D;
        double leftRightAvg = (Math.abs(BL_difference) + Math.abs(FR_difference)) / 2.0D;
        double rightLeftAvg = (Math.abs(FL_difference) + Math.abs(BR_difference)) / 2.0D;
        float allPower = allAvg <= SLOW_COUNTS ? (float) (allAvg * SPEED_SLOPE + SLOW_SPEED) : speed;
        float leftRightPower = leftRightAvg <= SLOW_COUNTS ? (float) (leftRightAvg * SPEED_SLOPE + SLOW_SPEED) : speed;
        float rightLeftPower = rightLeftAvg <= SLOW_COUNTS ? (float) (rightLeftAvg * SPEED_SLOPE + SLOW_SPEED) : speed;
        boolean done = false;

        switch (direction) {
            case FORWARD:
                hardware.omniDrive.moveForward(allPower);
                done = allAvg <= ENCODER_THRESHOLD || prevAllAvg < allAvg;
                break;
            case LEFT:
                hardware.omniDrive.moveLeft(allPower);
                done = allAvg <= ENCODER_THRESHOLD || prevAllAvg < allAvg;
                break;
            case RIGHT:
                hardware.omniDrive.moveRight(allPower);
                done = allAvg <= ENCODER_THRESHOLD || prevAllAvg < allAvg;
                break;
            case BACKWARD:
                hardware.omniDrive.moveBackward(allPower);
                done = allAvg <= ENCODER_THRESHOLD || prevAllAvg < allAvg;
                break;
            case FORWARD_LEFT:
                hardware.omniDrive.moveForwardLeft(rightLeftPower);
                done = rightLeftAvg <= ENCODER_THRESHOLD || prevRightLeftAvg < rightLeftAvg;
                break;
            case FORWARD_RIGHT:
                hardware.omniDrive.moveForwardRight(leftRightPower);
                done = leftRightAvg <= ENCODER_THRESHOLD || prevLeftRightAvg < leftRightAvg;
                break;
            case BACKWARD_LEFT:
                hardware.omniDrive.moveBackwardLeft(leftRightPower);
                done = leftRightAvg <= ENCODER_THRESHOLD || prevLeftRightAvg < leftRightAvg;
                break;
            case BACKWARD_RIGHT:
                hardware.omniDrive.moveBackwardRight(rightLeftPower);
                done = rightLeftAvg <= ENCODER_THRESHOLD || prevRightLeftAvg < rightLeftAvg;
                break;
            default:
                break;
        }

        if (done) {
            hardware.omniDrive.stopDrive();
            hardware.omniDrive.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.omniDrive.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.omniDrive.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.omniDrive.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        hardware.telemetry.addData("All Avg", allAvg);
        hardware.telemetry.addData("LR Avg", leftRightAvg);
        hardware.telemetry.addData("RL Avg", rightLeftAvg);

        prevAllAvg = allAvg;
        prevLeftRightAvg = leftRightAvg;
        prevRightLeftAvg = rightLeftAvg;

        return done;
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
