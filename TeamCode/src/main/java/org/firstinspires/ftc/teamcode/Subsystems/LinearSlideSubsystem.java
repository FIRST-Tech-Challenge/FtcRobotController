package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class LinearSlideSubsystem extends SubsystemBase {

    // Local objects and variables here


    public static final int top_linearslide_ticks = 1660; // adjusted to proper field height at T1 from memory

    public static final int mid_linearslide_ticks = 1000;

    public static final int low_linearslide_ticks = 500;

    public static final int bottom_linearslide_ticks = 0;

    private DcMotorEx leftLinearSlide;
    private DcMotorEx rightLinearSlide;

    /** Place code here to initialize subsystem */
    public LinearSlideSubsystem() {
        leftLinearSlide = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "left_linear_slide");
        rightLinearSlide = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "right_linear_slide");
        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    // place special subsystem methods here
    public void Movelinearslide(int ticks){
        leftLinearSlide.setTargetPosition(ticks);
        rightLinearSlide.setTargetPosition(ticks);
        leftLinearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightLinearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setPower(1);
        rightLinearSlide.setPower(1);
}
    public void LinearSlidesMiddle() {
        setPIDF(10.0, 0.2, 0.001, 10.0);
        Movelinearslide(mid_linearslide_ticks);
    }
    public void setPIDF (double p, double i,double d, double f){

        leftLinearSlide.setVelocityPIDFCoefficients(p, i, d, f);
        rightLinearSlide.setVelocityPIDFCoefficients(p, i, d, f);

    }

}