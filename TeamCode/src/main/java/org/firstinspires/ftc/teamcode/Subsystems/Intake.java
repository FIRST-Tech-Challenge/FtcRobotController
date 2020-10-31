package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotor intake_motor = null;

    
    
    
    //contstants
    public static final double intake_speed = .5;
    public static final double Intakeoff = 0;
    public static final double Intakeon = 0.9;

    // initialize intake
    public void init(HardwareMap hwMap){
        intake_motor =hwMap.get(DcMotor.class,"Intake");
        intake_motor.setDirection(DcMotor.Direction.FORWARD);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Intakeoff() {
        intake_motor.setPower(Intakeoff);

    }
    public void Intakeon() {
        intake_motor.setPower(Intakeon);

    }
    public void IntakeReverse(){
        intake_motor.setPower(-Intakeon);
    }
    public void Elevatorbackup() {
        intake_motor.setPower(-Intakeon);
    }

    public void IntakeReverse() {
        intake_motor.setPower(-Intakeon);
    }
}






