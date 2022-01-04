package org.firstinspires.ftc.teamcode;
/*
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Placing {

    //This class will contain anything having to do the foundation game element.
    //this includes the claw servos (wrist and grip) and the plate hook servos and the arm motor
    //Methods will include an open/close grip and hooks, rotate wrist, etc

    public Servo clawGrip   = null;
    public Servo clawWrist  = null;

    public DcMotor armMotor = null;
    //DigitalChannel REVTouchInside;

    public Servo rPlateHook = null;
    public Servo lPlateHook = null;

    HardwareMap hwMap       = null;
    private ElapsedTime period  = new ElapsedTime();
    double runtime = 0;

    public Placing(){
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize ALL installed servos
        clawGrip = hwMap.get(Servo.class, "grip_servo");
        clawWrist = hwMap.get(Servo.class, "wrist_servo");

        armMotor = hwMap.get(DcMotor.class, "arm_drive");
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //REVTouchInside = hwMap.get(DigitalChannel.class, "Inside_Touch");
        //REVTouchInside.setMode(DigitalChannel.Mode.INPUT);

        rPlateHook = hwMap.get(Servo.class, "right_hook");
        lPlateHook = hwMap.get(Servo.class, "left_hook");

        clawGrip.setPosition(.67);
        clawWrist.setPosition(.32);

        rPlateHook.setPosition(1);
        lPlateHook.setPosition(0);
    }

    //All servo values are subject to change

    public void setClawGrip(ServoPosition position){
        //lower bar to hold brick
        if (position == ServoPosition.DOWN){
            clawGrip.setPosition(.7);
            runtime = period.time();
            while (.6 > period.time() - runtime);
        }
        //raise bar to release brick
        if (position == ServoPosition.UP){
            clawGrip.setPosition(0);
            runtime = period.time();
            while (.6 > period.time() - runtime);
        }
    }
    public void setClawWrist(ServoPosition position){
        //raise gripper to clear lift
        if (position == ServoPosition.UP){
            clawWrist.setPosition(0);
            runtime = period.time();
            while (1 > period.time() - runtime);
        }
        //lower gripper
        if (position == ServoPosition.DOWN){
            clawWrist.setPosition(.77);
            runtime = period.time();
            while (1 > period.time() - runtime);
        }
    }
    public void setPlateHooks(ServoPosition position){
        //lower hooks onto plate so it can be moved
        if (position == ServoPosition.DOWN){
            rPlateHook.setPosition(0);
            lPlateHook.setPosition(1);
            runtime = period.time();
            while (.6 > period.time() - runtime);
        }
        //raise hooks off plate
        if (position == ServoPosition.UP){
            rPlateHook.setPosition(1);
            lPlateHook.setPosition(0);
            runtime = period.time();
            while (.6 > period.time() - runtime);
        }
    }

    /*
    //brings arm in to the touch sensor
    public void armRecall(double power){
        if (REVTouchInside.getState()){
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armMotor.setPower(-power);
            while (REVTouchInside.getState());
            armMotor.setPower(0);

            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

     *//*
    public void armDrive(double speed, int encoderDistance){
        armMotor.setTargetPosition(encoderDistance);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(speed);
        while (armMotor.isBusy());
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double getArmEncoder(){
        return armMotor.getCurrentPosition();
    }
}
*/