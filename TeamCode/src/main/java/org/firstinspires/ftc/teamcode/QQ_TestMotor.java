package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class QQ_TestMotor extends QQ_Test {
    private double speed;
    private DcMotor motor;


    /**
     * @param description string that describes the test
     * @param speed       the speed for the test to use
     * @param motor       the motor for the test to use
     */
    QQ_TestMotor(String description, double speed, DcMotor motor) {
        super(description);
        this.speed = speed;
        this.motor = motor;
    }

    /**
     * spins the motor at set speed or turns it off
     *
     * @param on        determines the action taken -- true = spin motor; false = stop
     * @param telemetry allows for sending encoder ticks back so we know if the encoder is wired right
     */
    @Override
    public void run(boolean on, Telemetry telemetry) {
        if (on) {
            motor.setPower(speed);
        } else {
            motor.setPower(0.0);
        }
        telemetry.addData("Encoders:", motor.getCurrentPosition());
    }
}