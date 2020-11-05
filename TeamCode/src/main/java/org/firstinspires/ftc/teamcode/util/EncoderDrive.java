package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

/**
 * Created by djfigs1 on 9/30/17. not really
 */

public class EncoderDrive {

    private RobotHardware robotHardware;
    private OmniDrive omniDrive;
    public boolean isBusy = false;

    private double inchesToDrive;
    private OmniDrive.Direction direction;
    private DcMotor.RunMode previousRunMode;
    private double timeout;
    private ElapsedTime runTime;

    private int FL_targetPosition;
    private int FR_targetPosition;
    private int BL_targetPosition;
    private int BR_targetPosition;

    private float FL_speed;
    private float FR_speed;
    private float BL_speed;
    private float BR_speed;

    public class Distance {

        double value;
        DistanceUnit unit;

        public Distance(double value, DistanceUnit unit) {
            this.value = value;
            this.unit = unit;

            switch (unit) {

                case METER:
                    break;
                case CM:
                    break;
                case MM:
                    break;
                case INCH:
                    break;
            }
        }

    }

    public EncoderDrive(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
        this.omniDrive = robotHardware.omniDrive;
    }

    /**
     * This function prepares the EncoderDrive to run
     * @param direction Which direction the robot will move
     * @param distance The distance (in centimeters) the robot will move
     * @param power How much power is applied to each motor (0 -> 1)
     * @param timeout Timeout in case something should go wrong
     */

    public void setInchesToDrive(OmniDrive.Direction direction, double distance, float power, double timeout) {
        this.inchesToDrive = distance;
        this.isBusy = true;
        this.timeout = timeout;
        this.runTime = new ElapsedTime();
        this.runTime.reset();

        this.previousRunMode = omniDrive.frontLeft.getMode();

        switch (direction) {
            case FORWARD:
                FL_speed = power;
                FR_speed = power;
                BL_speed = power;
                BR_speed = power;
                break;

            case BACKWARD:
                FL_speed = -power;
                FR_speed = -power;
                BL_speed = -power;
                BR_speed = -power;
                break;

            case LEFT:
                FL_speed = -power;
                FR_speed = power;
                BL_speed = power;
                BR_speed = -power;
                break;

            case RIGHT:
                FL_speed = power;
                FR_speed = -power;
                BL_speed = -power;
                BR_speed = power;
                break;

            case FORWARD_LEFT:
                FL_speed = 0;
                FR_speed = power;
                BL_speed = power;
                BR_speed = 0;
                break;

            case FORWARD_RIGHT:
                FL_speed = power;
                FR_speed = 0;
                BL_speed = 0;
                BR_speed = power;
                break;

            case BACKWARD_LEFT:
                FL_speed = 0;
                FR_speed = -power;
                BL_speed = -power;
                BR_speed = 0;
                break;

            case BACKWARD_RIGHT:
                FL_speed = -power;
                FR_speed = 0;
                BL_speed = 0;
                BR_speed = -power;
                break;

            case ROTATE_LEFT:
                FL_speed = -power;
                FR_speed = power;
                BL_speed = -power;
                BR_speed = power;
                break;

            case ROTATE_RIGHT:
                FL_speed = power;
                FR_speed = -power;
                BL_speed = power;
                BR_speed = -power;
                break;

        }

        int FL_direction = (FL_speed > 0) ? 1 : (FL_speed < 0) ? -1 : 0;
        int FR_direction = (FR_speed > 0) ? 1 : (FR_speed < 0) ? -1 : 0;
        int BL_direction = (BL_speed > 0) ? 1 : (BL_speed < 0) ? -1 : 0;
        int BR_direction = (BR_speed > 0) ? 1 : (BR_speed < 0) ? -1 : 0;

        if (direction == OmniDrive.Direction.LEFT || direction == OmniDrive.Direction.RIGHT) {
            FL_targetPosition = omniDrive.frontLeft.getCurrentPosition() + FL_direction * (int)(robotHardware.omniDrive.getCountsPerInch() * distance);
            FR_targetPosition = omniDrive.frontRight.getCurrentPosition() + FR_direction * (int)(robotHardware.omniDrive.getCountsPerInch()  * distance);
            BL_targetPosition = omniDrive.backLeft.getCurrentPosition() + BL_direction * (int)(robotHardware.omniDrive.getCountsPerInch() * distance);
            BR_targetPosition = omniDrive.backRight.getCurrentPosition() + BR_direction * (int)(robotHardware.omniDrive.getCountsPerInch() * distance);
        } else {
            FL_targetPosition = omniDrive.frontLeft.getCurrentPosition() + FL_direction * (int)(robotHardware.omniDrive.getCountsPerInch() * distance);
            FR_targetPosition = omniDrive.frontRight.getCurrentPosition() + FR_direction * (int)(robotHardware.omniDrive.getCountsPerInch() * distance);
            BL_targetPosition = omniDrive.backLeft.getCurrentPosition() + BL_direction * (int)(robotHardware.omniDrive.getCountsPerInch() * distance);
            BR_targetPosition = omniDrive.backRight.getCurrentPosition() + BR_direction * (int)(robotHardware.omniDrive.getCountsPerInch() * distance);
        }



        omniDrive.frontLeft.setTargetPosition(FL_targetPosition);
        omniDrive.frontRight.setTargetPosition(FR_targetPosition);
        omniDrive.backLeft.setTargetPosition(BL_targetPosition);
        omniDrive.backRight.setTargetPosition(BR_targetPosition);

        omniDrive.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniDrive.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniDrive.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniDrive.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setCountsToDrive(OmniDrive.Direction direction, int counts, float power, double timeout) {
        this.isBusy = true;
        this.timeout = timeout;
        this.runTime = new ElapsedTime();
        this.runTime.reset();

        this.previousRunMode = omniDrive.frontLeft.getMode();

        switch (direction) {
            case FORWARD:
                FL_speed = power;
                FR_speed = power;
                BL_speed = power;
                BR_speed = power;
                break;

            case BACKWARD:
                FL_speed = -power;
                FR_speed = -power;
                BL_speed = -power;
                BR_speed = -power;
                break;

            case LEFT:
                FL_speed = -power;
                FR_speed = power;
                BL_speed = power;
                BR_speed = -power;
                break;

            case RIGHT:
                FL_speed = power;
                FR_speed = -power;
                BL_speed = -power;
                BR_speed = power;
                break;

            case FORWARD_LEFT:
                FL_speed = 0;
                FR_speed = power;
                BL_speed = power;
                BR_speed = 0;
                break;

            case FORWARD_RIGHT:
                FL_speed = power;
                FR_speed = 0;
                BL_speed = 0;
                BR_speed = power;
                break;

            case BACKWARD_LEFT:
                FL_speed = 0;
                FR_speed = -power;
                BL_speed = -power;
                BR_speed = 0;
                break;

            case BACKWARD_RIGHT:
                FL_speed = -power;
                FR_speed = 0;
                BL_speed = 0;
                BR_speed = -power;
                break;

            case ROTATE_LEFT:
                FL_speed = -power;
                FR_speed = power;
                BL_speed = -power;
                BR_speed = power;
                break;

            case ROTATE_RIGHT:
                FL_speed = power;
                FR_speed = -power;
                BL_speed = power;
                BR_speed = -power;
                break;

        }

        int FL_direction = (FL_speed > 0) ? 1 : (FL_speed < 0) ? -1 : 0;
        int FR_direction = (FR_speed > 0) ? 1 : (FR_speed < 0) ? -1 : 0;
        int BL_direction = (BL_speed > 0) ? 1 : (BL_speed < 0) ? -1 : 0;
        int BR_direction = (BR_speed > 0) ? 1 : (BR_speed < 0) ? -1 : 0;

        FL_targetPosition = omniDrive.frontLeft.getCurrentPosition() + FL_direction * counts;
        FR_targetPosition = omniDrive.frontRight.getCurrentPosition() + FR_direction * counts;
        BL_targetPosition = omniDrive.backLeft.getCurrentPosition() + BL_direction * counts;
        BR_targetPosition = omniDrive.backRight.getCurrentPosition() + BR_direction * counts;

        omniDrive.frontLeft.setTargetPosition(FL_targetPosition);
        omniDrive.frontRight.setTargetPosition(FR_targetPosition);
        omniDrive.backLeft.setTargetPosition(BL_targetPosition);
        omniDrive.backRight.setTargetPosition(BR_targetPosition);

        omniDrive.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniDrive.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniDrive.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniDrive.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void run(RobotHardware hardware) {
        if (this.isBusy) {
            boolean busy = omniDrive.frontLeft.isBusy() && omniDrive.frontRight.isBusy()
                    && omniDrive.backLeft.isBusy() && omniDrive.backRight.isBusy();

            if (busy && this.runTime.milliseconds() <= this.timeout) {
                omniDrive.frontLeft.setPower(FL_speed);
                omniDrive.frontRight.setPower(FR_speed);
                omniDrive.backLeft.setPower(BL_speed);
                omniDrive.backRight.setPower(BR_speed);

                hardware.telemetry.addData("FL", String.format("%d -> %d", omniDrive.frontLeft.getCurrentPosition(), omniDrive.frontLeft.getTargetPosition()));
                hardware.telemetry.addData("FR", String.format("%d -> %d", omniDrive.frontRight.getCurrentPosition(), omniDrive.frontRight.getTargetPosition()));
                hardware.telemetry.addData("BL", String.format("%d -> %d", omniDrive.backLeft.getCurrentPosition(), omniDrive.backLeft.getTargetPosition()));
                hardware.telemetry.addData("BR", String.format("%d -> %d", omniDrive.backRight.getCurrentPosition(), omniDrive.backRight.getTargetPosition()));
            } else {
                isBusy = false;
                omniDrive.stopDrive();

                omniDrive.frontLeft.setMode(previousRunMode);
                omniDrive.frontRight.setMode(previousRunMode);
                omniDrive.backLeft.setMode(previousRunMode);
                omniDrive.backRight.setMode(previousRunMode);
            }

        }
    }
}
