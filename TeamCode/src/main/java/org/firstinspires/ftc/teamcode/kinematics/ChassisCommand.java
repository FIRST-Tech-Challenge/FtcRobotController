package org.firstinspires.ftc.teamcode.kinematics;

/** Data class storing the wheel speeds for the left-front, right-front, left-rear, and right-rear motors
 * @author TheConverseEngineer
 */
public class ChassisCommand {

    // Order is lf, rf, lr, rr
    public double[] speeds;

    public ChassisCommand(double lf,double rf,double lr,double rr) {
        this.speeds[0] = lf;
        this.speeds[1] = rf;
        this.speeds[2] = lr;
        this.speeds[3] = rr;
    }

    public ChassisCommand() {
        this(0, 0, 0, 0);
    }

    public ChassisCommand normalize() {
        double max = Math.max(Math.max(Math.abs(this.speeds[0]), Math.abs(this.speeds[1])), Math.max(Math.abs(this.speeds[2]), Math.abs(this.speeds[3])));
        for (double i : speeds) {
            i /= max;
        }
        return this;
    }

    public ChassisCommand scale(double scalar) {
        for (double i : speeds) {
            i *= scalar;
        }
        return this;
    }

    public double getLF() { return speeds[0]; }
    public double getRF() { return speeds[1]; }
    public double getLR() { return speeds[2]; }
    public double getRR() { return speeds[3]; }

}
