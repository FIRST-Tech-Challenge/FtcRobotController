package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Usefuls.Math.M;
import org.firstinspires.ftc.teamcode.Usefuls.Motor.PID;


@Config
public class Slides {
    private static final double TICKS_TO_INCHES = 0.0287253141831239; //
    public double targetLinSlidePosition = 0;
    public static double sKp = .6, sKi = 0, sKd = 0.020;
    private final DcMotorEx s;
    private final PID linSlideController;
    private final DcMotorEx slidesEncoder;
    public static double aPower = .6;
    public static double bPower = -1;
    public Slides(HardwareMap hardwareMap, DcMotorEx slidesEncoder) {
        s = hardwareMap.get(DcMotorEx.class, "slides");
        this.slidesEncoder = slidesEncoder;

        this.linSlideController = new PID(new PID.Coefficients(sKp, sKi, sKd), () -> (this.getCurrentSlidesPosition()) - targetLinSlidePosition, factor -> {
            this.s.setPower(M.clamp(factor, aPower, bPower)); //b is extension
        });
    }


    public void resetEncoder() {
        slidesEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getError() {
        return getCurrentSlidesPosition() - targetLinSlidePosition;
    }

    public double getCurrentSlidesPosition() {
        return this.slidesEncoder.getCurrentPosition() * TICKS_TO_INCHES;
    }

    /**
     * sets the position of the slides in inches
     *
     * @param position the position in inches
     */
    public void setTargetSlidesPosition(double position) {
        targetLinSlidePosition = position;
    }

    public void floorIntake() {
        setTargetSlidesPosition(0.25);
    }

    public void preScore() {
        setTargetSlidesPosition(2.25);
    }

    public void score() {
        setTargetSlidesPosition(29);
    }

    public void update() {
        linSlideController.update();
    }


}