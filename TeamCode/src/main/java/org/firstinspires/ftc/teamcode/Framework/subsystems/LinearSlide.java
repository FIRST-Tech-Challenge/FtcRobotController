package org.firstinspires.ftc.teamcode.Framework.subsystems;

import android.transition.Slide;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideConstants;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideState;

public class LinearSlide extends SubsystemBase {
    private HardwareMap hw;
    private String name;
    private MotorEx slideMotor;
    private SlideState state;
    private PIDFController controller;

    public LinearSlide(final HardwareMap hw, final String name){
        this.hw = hw;
        this.name = name;
        this.state = SlideState.DOWN;
        this.controller = new PIDFController(SlideConstants.KP, SlideConstants.KI, SlideConstants.KD, SlideConstants.KF);

        slideMotor = new MotorEx(hw, "slideMotor");
    }

    public void setState(SlideState state){
        this.state = state;
    }

    @Override
    public void periodic(){
        double output = controller.calculate(slideMotor.getCurrentPosition(), SlideConstants.getTicks(state));
        slideMotor.setVelocity(output);
    }
}
