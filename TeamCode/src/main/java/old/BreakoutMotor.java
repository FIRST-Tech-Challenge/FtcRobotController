package old;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This class is used to simplify the usage of DC Motors in the code.
 * Please use this class instead of DcMotor.
 */
public class BreakoutMotor {

    //Enum for direction
    public enum Direction {
        MOTOR_F(DcMotor.Direction.FORWARD),
        MOTOR_R(DcMotor.Direction.REVERSE);

        private DcMotor.Direction motorDirection;

        Direction(DcMotor.Direction motorDirection) {
            this.motorDirection = motorDirection;
        }
    }

    //Define motor as a DcMotor
    private DcMotor motor;

    //Retrieve motor var
    public DcMotor get() {
        return motor;
    }

    //Set hardware map data of motor
    public void set(DcMotor motor) {
        this.motor = motor;
    }

    //Set the direction the motor turns in (FORWARD or REVERSE)
    public void setDirection(Direction direction) {
        motor.setDirection(direction.motorDirection);
    }

    //Set the power the motor turns at (0-1)
    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setMotorMode(DcMotor.RunMode mode) {
        this.motor.setMode(mode);
    }

    public void setTargetPosition(int pos) { this.motor.setTargetPosition(pos); }

    public int getCurrentPosition() { return this.motor.getCurrentPosition(); }

    public boolean isBusy() { return this.motor.isBusy(); }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public double getPower() { return this.motor.getPower(); }
}
