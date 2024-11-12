package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CustomPID;

public class Turn {
    private DcMotor dlift;

    private final double INCHTOENCH = 115.35;
    private final double TURNMAX = 1600;

    public void init(HardwareMap hwMap){
        this.dlift = hwMap.get(DcMotor.class, "dlift");
        this.dlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.dlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.dlift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power){
        this.dlift.setPower(power);
    }
    public double getPos(){
        return this.dlift.getCurrentPosition();
    }
    public void turn(double gamepadInput){
        if (Math.abs(this.dlift.getCurrentPosition()) < TURNMAX) {
            this.dlift.setPower(-0.2 * gamepadInput);
        } else {
            this.dlift.setPower(-0.05);
        }
    }

    public void gotoMaxPosition(double percent){
        CustomPID c1 = new CustomPID(new double[]{.000001, 0.00004, 0.00001});
        c1.setSetpoint(-TURNMAX * percent);
        while (((-TURNMAX * percent)-this.dlift.getCurrentPosition()) > 35){
            double[] outputs = c1.calculateGivenRaw(this.dlift.getCurrentPosition());
            double power = outputs[0];
            this.dlift.setPower(power);
        }
        this.dlift.setPower(0.0);
    }

}
