package org.firstinspires.ftc.forteaching;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SwerveController {
    // At 4.8v, no load 60 degrees takes super-speed servos .055 seconds to move
    // They're faster at more volts, so 195ms + an extra 55mm for "load" consideration:
    private static int DIR_TRANSITION_DELAY_MS = 250;
    // The range for the non-CRServo configuration to give,
    // as precisely as possible, a 180 degree distances from 0.0 to 1.0
    private static double SERVO_LO_SCALE = 1.0 / 6.0;
    private static double SERVO_HI_SCALE = 5.0 / 6.0;
    // In case people are wondering, division is slow. Floating point division is even slower
    // Invert the values, and multiply instead. (This is probably silly, but I don't know if the
    // device has an actual FPU or not. If not, multiplying is *WAY* faster, even if it does,
    // it's still faster)
    private static double INV_PI = 1.0 / Math.PI;
    private static double INV_180 = 1.0 / 180.0;

    private DcMotorEx mfl, mfr, mrl, mrr;
    // A bunch of stuff gets *much* easier when we can use CRServo's with encoders :)
    private Servo sfl, sfr, srl, srr;
    // Stuff to remember if the servo/motor are in "flipped" mode or not
    // Not needed once we have encoders: Just use CRServo's & encoders instead...
    private boolean flFlip, frFlip, rlFlip, rrFlip;

    private static void Delay() {
        try {
            Thread.sleep(DIR_TRANSITION_DELAY_MS);
        } catch (Exception e) {
        }
    }

    // Force a wrap-around value (like degrees or radians) to be within low/high
    private static double inRange(double low, double high, double val) {
        // Dumb & slow, but easy, and visually correct...
        while (val < low) {
            val += (high - low);
        }
        while (val >= high) {
            val -= (high - low);
        }
        return val;
    }

    // This is just to make it easy if I didn't think of this properly
    // A negative angle needs to be flipped around
    // Anglish: -1 to 1 is -180 to 180 (or -pi to pi)
    static boolean needFlip(double anglish) {
        return anglish < 0.0;
    }
    // This 'flips' the angle. I _think_ this makes sense.
    // To move at .5 power, at -45 degrees is the same as moving at -.5 power at 135 degrees
    // so you just "add" 180 degrees (1 in "anglish" units)
    static double flip(double anglish) {
        // assert(anglish <= 0.0);
        return anglish + 1.0;
    }
    // The angles for this function are a scale from -1 to +1 meaning -180 degrees to +180 degrees
    // But since we can only move 180 degrees, we may have to invert the wheel motor...
    private void setControlRelative(double flp, double fla, double frp, double fra, double rlp, double rla, double rrp, double rra) {
        // See if *any* of the servos require 'flipping'
        boolean ffl = (needFlip(fla) != flFlip);
        boolean ffr = (needFlip(fra) != frFlip);
        boolean frl = (needFlip(rla) != rlFlip);
        boolean frr = (needFlip(rra) != rrFlip);
        boolean somethingFlipping = (ffl || ffr || frl || frr);
        if (somethingFlipping) {
            // If we're flipping a servo, we have to stop the bot so it doesn't freak out
            // while the servos are moving a *lot*
            stop();
            flFlip = needFlip(fla);
            frFlip = needFlip(fra);
            rlFlip = needFlip(rla);
            rrFlip = needFlip(rra);
        }
        // Set all 4 servos to the correct position, dealing with 'flips'
        if (flFlip) fla = flip(fla);
        if (frFlip) fra = flip(fra);
        if (rlFlip) rla = flip(rla);
        if (rrFlip) rra = flip(rra);
        // Now set all the servo's angles
        sfl.setPosition(fla);
        sfr.setPosition(fra);
        srl.setPosition(rla);
        srr.setPosition(rra);
        // We need to wait until the wheels get to the right point, now
        if (somethingFlipping) Delay();
        // Now, make the motors go in the right direction :)
        mfl.setPower(flFlip ? -flp : flp);
        mfr.setPower(frFlip ? -frp : frp);
        mrl.setPower(rlFlip ? -rlp : rlp);
        mrr.setPower(rrFlip ? -rrp : rrp);
    }

    public SwerveController(DcMotorEx flm, DcMotorEx frm, DcMotorEx rlm, DcMotorEx rrm, Servo fls, Servo frs, Servo rls, Servo rrs) {
        mfl = flm;
        mfr = frm;
        mrl = rlm;
        mrr = rrm;
        sfl = fls;
        sfr = frs;
        srl = rls;
        srr = rrs;

        // Until we get encoders, we can use the servo's in normal mode
        sfl.scaleRange(SERVO_LO_SCALE, SERVO_HI_SCALE);
        sfr.scaleRange(SERVO_LO_SCALE, SERVO_HI_SCALE);
        srl.scaleRange(SERVO_LO_SCALE, SERVO_HI_SCALE);
        srr.scaleRange(SERVO_LO_SCALE, SERVO_HI_SCALE);

        // More crap dealing with servos instead of CRServos
        flFlip = false;
        frFlip = false;
        rlFlip = false;
        rrFlip = false;
    }

    public void setControlRadians(double flp, double fla, double frp, double fra, double rlp, double rla, double rrp, double rra) {
        fra = inRange(-Math.PI, Math.PI, fra) * INV_PI;
        fla = inRange(-Math.PI, Math.PI, fla) * INV_PI;
        rra = inRange(-Math.PI, Math.PI, rra) * INV_PI;
        rla = inRange(-Math.PI, Math.PI, rla) * INV_PI;
        setControlRelative(flp, fla, frp, fra, rlp, rla, rrp, rra);
    }

    public void setControlDegrees(double flp, double fla, double frp, double fra, double rlp, double rla, double rrp, double rra) {
        fra = inRange(-180, 180, fra) * INV_180;
        fla = inRange(-180, 180, fla) * INV_180;
        rra = inRange(-180, 180, rra) * INV_180;
        rla = inRange(-180, 180, rla) * INV_180;
        setControlRelative(flp, fla, frp, fra, rlp, rla, rrp, rra);
    }

    public void stop() {
        mfl.setPower(0);
        mfr.setPower(0);
        mrl.setPower(0);
        mrr.setPower(0);
    }
}
