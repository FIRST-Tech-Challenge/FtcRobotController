package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DummyBot {
    public DcMotor motor1 = null;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;
    public DcMotor motor4 = null;

    HardwareMap hwMap =  null;

    private Servo wobbleSwing = null;
    private Servo wobbleClaw = null;

    /* Constructor */
    public DummyBot(){

    }

    public void init(HardwareMap ahwMap, Telemetry telemetry) throws Exception {

        hwMap = ahwMap;

        try {
            motor1 = hwMap.get(DcMotor.class, "motor1");
            motor1.setDirection(DcMotor.Direction.FORWARD);
            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor1.setPower(0);
        } catch (Exception ex) {
            throw new Exception("Issues motor 1. Check the controller config", ex);
        }

        try{
            motor2 = hwMap.get(DcMotor.class, "motor2");
            motor2.setDirection(DcMotor.Direction.FORWARD);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setPower(0);
        }
        catch (Exception ex) {
            throw new Exception("Issues with motor 2. Check the controller config", ex);
        }

        try{
            motor3 = hwMap.get(DcMotor.class, "motor3");
            motor3.setDirection(DcMotor.Direction.FORWARD);
            motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor3.setPower(0);
        }
        catch (Exception ex) {
            throw new Exception("Issues with motor 3. Check the controller config", ex);
        }

        try{
            motor4 = hwMap.get(DcMotor.class, "motor4");
            motor4.setDirection(DcMotor.Direction.FORWARD);
            motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor4.setPower(0);
        }
        catch (Exception ex) {
            throw new Exception("Issues with motor 4. Check the controller config", ex);
        }

        telemetry.addData("Init", "Dummy is ready");
    }


    public void moveMotor1(double speed){
        double power = Range.clip(speed, -1.0, 1.0);

        motor1.setPower(power);
    }

    public void moveMotor2(double speed){
        double power = Range.clip(speed, -1.0, 1.0);

        motor2.setPower(power);
    }

    public void moveMotor3(double speed){
        double power = Range.clip(speed, -1.0, 1.0);

        motor3.setPower(power);
    }

    public void moveMotor4(double speed){
        double power = Range.clip(speed, -1.0, 1.0);

        motor4.setPower(power);
    }

    public void moveWobbleSwing (double position) {
        double p = Range.clip(position, -1.0, 1.0);

        wobbleSwing.setPosition(p);

    }

    public void moveWobbleClaw (double position) {
        double p = Range.clip(position, -1.0, 1.0);

        wobbleClaw.setPosition(p);

    }



}
