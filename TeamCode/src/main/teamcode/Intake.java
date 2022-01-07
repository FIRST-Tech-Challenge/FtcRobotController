package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    public DcMotor take;

    private final ElapsedTime timerIntake = new ElapsedTime();
    public final LinearOpMode intake;

    public Intake(LinearOpMode intake){
        take = intake.hardwareMap.dcMotor.get("take");
        take.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.intake=intake;
    }

    int intakePos(){
        return (take.getCurrentPosition());
    }

    public void intaking(double voom){
        if(voom > .3){
            take.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            take.setPower(-.7);
        } else if(voom < -.3) {
            take.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            take.setPower(.7);
        } else {
            take.setPower(0);
        }
    }
}