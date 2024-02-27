package org.firstinspires.ftc.teamcode.robotParts;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CustomPID;

public class LinearLift {
    private DcMotor lift;
    private final double INCHTOENCH = 115.35;

    private final double LINMAX = 3900;

    public void init(HardwareMap hwMap){
        this.lift = hwMap.get(DcMotor.class, "lift");
        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setPower(double power){
        this.lift.setPower(power);
    }
    public double getPos(){
        return lift.getCurrentPosition();
    }
    public void gotoPosition(double positionInches){
        CustomPID c1 = new CustomPID(new double[]{.000001, 0.00004, 0.00001});
        c1.setSetpoint(positionInches * INCHTOENCH+17);
        double[] outputs = c1.calculateGivenRaw(lift.getCurrentPosition());
        double power = outputs[0];
        lift.setPower(power);
    }
    public void moveLift(double gamepadInput){
        if (Math.abs(lift.getCurrentPosition()) < LINMAX) {
            lift.setPower(-1 * gamepadInput);
        } else {
            lift.setPower(-0.2);
        }
    }
    public DcMotor getLift() {
        return lift;
    }
}
