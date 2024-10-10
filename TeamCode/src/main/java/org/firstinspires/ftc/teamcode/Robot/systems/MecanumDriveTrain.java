package org.firstinspires.ftc.teamcode.Robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class MecanumDriveTrain {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    double speedMultiplier = 1;

    private boolean fieldCentricDriving = true;
    
    /**
     * constructor for mecanum drive train
     * -assigns motors from hardware map
     * -sets directions
     * -sets zero power behavior to brake
     * @param hardwareMap hardware map of robot
     */
    public MecanumDriveTrain(HardwareMap hardwareMap) {

        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * used to change whether or not you use feedback from encoders
     * to keep the motors at the correct speed
     *
     * @param velocityMode the boolean that tells if you want to use "RUN_USING_ENCODER" mode
     */
    public void setVelocityMode(boolean velocityMode) {

        if (velocityMode) {

            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else {

            motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

    /**
     * Sets the speed multiplier
     *
     * @param speedMultiplier speed multiplier (between 0 and 1)
     */
    public void setSpeedMultiplier(double speedMultiplier) {

        // ensure valid speed multiplier
        if (speedMultiplier < 0 || speedMultiplier > 1) {

            speedMultiplier = 1;

        }

        // sets speed multiplier
        this.speedMultiplier = speedMultiplier;

    }

    /**
     * sets the motor powers
     *
     * @param forwardPower  amount it goes forward
     * @param sidewaysPower amount it goes sideways
     * @param turnPower     amount it turns
     * @param robotHeading  yaw angle of robot
     */
    public void setDrivePower(double forwardPower, double sidewaysPower, double turnPower, double robotHeading) {
        // Field Centric Driving aligns all robot movements with the player's perspective from the field, rather than the robot's
        if (fieldCentricDriving) {
            double temp = forwardPower * Math.cos(-robotHeading) + sidewaysPower * Math.sin(-robotHeading);
            sidewaysPower = -forwardPower * Math.sin(-robotHeading) + sidewaysPower * Math.cos(-robotHeading);
            forwardPower = temp;
        }
        
        motorFL.setPower(Range.clip(forwardPower + sidewaysPower - turnPower, -1.0, 1.0) * speedMultiplier);
        motorFR.setPower(Range.clip(forwardPower - sidewaysPower + turnPower, -1.0, 1.0) * speedMultiplier);
        motorBL.setPower(Range.clip(forwardPower - sidewaysPower - turnPower, -1.0, 1.0) * speedMultiplier);
        motorBR.setPower(Range.clip(forwardPower + sidewaysPower + turnPower, -1.0, 1.0) * speedMultiplier);
    }
}


