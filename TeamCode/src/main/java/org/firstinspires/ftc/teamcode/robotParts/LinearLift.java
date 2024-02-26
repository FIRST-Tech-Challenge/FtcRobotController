package org.firstinspires.ftc.teamcode.robotParts;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CustomPID;

public class LinearLift {
    private DcMotor lift;
    private final double ENCODERTOINCHES = 155.35;

    private final double LINMAX = 3900;

    public void init(HardwareMap hwMap){
        this.lift = hwMap.get(DcMotor.class, "lift");
        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setPower(double power){
        this.lift.setPower(power);
    }
    public double getPos(){
        return lift.getCurrentPosition();
    }
    public void gotoPosition(double positionInches){
        CustomPID liftController = new CustomPID(new double[]{.000001, 0.00004, 0.00001});
        double range = 50;
        while(Math.abs(lift.getCurrentPosition()) - (positionInches / this.ENCODERTOINCHES) <= range / 2.0){
            liftController.setSetpoint(positionInches / this.ENCODERTOINCHES);
            double[] outputs = liftController.calculateGivenRaw(lift.getCurrentPosition());
            double power = outputs[0];
            lift.setPower(power);
        }
        lift.setPower(0);
    }
    public void moveLift(double gamepadInput) throws InterruptedException {
        if (Math.abs(lift.getCurrentPosition()) < LINMAX) {
            if (gamepadInput != 0) {
                if(Math.abs(lift.getCurrentPosition()) < LINMAX) {
                    lift.setPower(-1 * gamepadInput);
                }
                lift.setPower(0);
            }
        } else {
            lift.setPower(0);
            lift.setPower(-0.2);
            sleep(100);
            lift.setPower(0);
        }
    }
    public DcMotor getLift() {
        return lift;
    }
}
