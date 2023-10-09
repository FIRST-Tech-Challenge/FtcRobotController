package org.firstinspires.ftc.teamcode.src.v2.Subsystem;

//import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.src.v2.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.src.v2.maths.mathsOperations;

public class SwerveModule {

    private final AnalogInput ma3;
    private final MotorGroup motors;
    private double moduleHeadingAdjustment;
    PIDcontroller PID = new PIDcontroller(0.2, 0.003, 0.01, 0.75, 100);

    public SwerveModule(AnalogInput ma3, DcMotorEx motor1, DcMotorEx motor2, double adjust) {
        this.ma3 = ma3;
        motors = new MotorGroup(motor1, motor2);
        this.moduleHeadingAdjustment = adjust;
    }

    public double getModuleHeading() {
        return mathsOperations.angleWrap(ma3.getVoltage() * 74.16 - moduleHeadingAdjustment);
    }

    public void adjustModule(double newAdjustment) {
        moduleHeadingAdjustment = newAdjustment;
    }

    public void runModule(double heading, double power) {
        if (mathsOperations.dynamicTurn(heading - getModuleHeading())) {
            heading -= 180;
            power *= -1;
        }
        motors.setPower(mathsOperations.diffyConvert(PID.pidOut(AngleUnit.normalizeDegrees(heading - getModuleHeading())), power)[0], 0);
        motors.setPower(mathsOperations.diffyConvert(PID.pidOut(AngleUnit.normalizeDegrees(heading - getModuleHeading())), power)[1], 1);
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf, double limit) {
        PID.setPIDgains(Kp, Kd, Ki, Kf, limit);
    }

}