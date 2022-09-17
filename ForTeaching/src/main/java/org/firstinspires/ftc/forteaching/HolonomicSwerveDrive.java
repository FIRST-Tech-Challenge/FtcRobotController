package org.firstinspires.ftc.forteaching;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

// There's a bunch of good stuff here:
// https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
// This one has all the details I'm implementing:
// https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
public class HolonomicSwerveDrive {
    private SwerveController ctrl;
    // L is the vehicle's wheelbase
    private static double L = 12;
    // W is the vehicle's trackwidth
    private static double W = 12;
    // R = sqrt(L*L+W*W);
    private static double R_INV = 1.0 / Math.sqrt(L * L + W * W);

    public HolonomicSwerveDrive(DcMotorEx flm, DcMotorEx frm, DcMotorEx rlm, DcMotorEx rrm, Servo fls, Servo frs, Servo rls, Servo rrs) {
        ctrl = new SwerveController(flm, frm, rlm, rrm, fls, frs, rls, rrs);
    }

    // ly, lx, rx(ish), plus the angle of the bot currently
    // TODO:
    // The imu's angle is potentially "off" so needs to be corrected for it's relative position
    public void setControlDataRelative(double forward, double strafe, double rotation, double imuAngle) {
        double relForward = forward * Math.cos(imuAngle) + strafe * Math.sin(imuAngle);
        double relStrafe = -forward * Math.sin(imuAngle) + strafe * Math.cos(imuAngle);
        setControlData(relForward, relStrafe, rotation);
    }

    // ly, lx, rx(ish)
    public void setControlData(double forward, double strafe, double rotation) {
        double A = strafe - rotation * L * R_INV;
        double B = strafe + rotation * L * R_INV;
        double C = forward - rotation * W * R_INV;
        double D = forward + rotation * W * R_INV;
        double sfr = Math.sqrt(B * B + C * C);
        double afr = Math.atan2(B, C);
        double sfl = Math.sqrt(B * B + D * D);
        double afl = Math.atan2(B, D);
        double srl = Math.sqrt(A * A + D * D);
        double arl = Math.atan2(A, D);
        double srr = Math.sqrt(A * A + C * C);
        double arr = Math.atan2(A, C);
        double max = Math.max(Math.max(Math.abs(sfr), Math.abs(sfl)),
                Math.max(Math.abs(srr), Math.abs(srl)));
        if (max > 1) {
            sfr /= max;
            sfl /= max;
            srr /= max;
            srl /= max;
        }
        ctrl.setControlRadians(sfl, afl, sfr, afr, srl, arl, srr, arr);
    }
}
