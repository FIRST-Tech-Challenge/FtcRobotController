package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.PDFLController;



@Config
@Disabled
public class Example1MotorLift extends SubsystemBase {


    private final DcMotorEx liftMotor;

    public static int liftTarget = 0;

    public static int P = 0, D = 0, F = 0, L = 0;

    PDFLController pdflController;
    public static double power;
    public static int allowedError = 1;




    public Example1MotorLift(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftMotor.setDirection(DcMotorEx.Direction.FORWARD);

        resetMotor();

    }

    public void resetMotor(){
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLiftTarget(int target){
        liftTarget = target;
    }

    public void setAllowedError(int error){
        allowedError = error;
    }

    public void update(){
        pdflController.setPDFL(P, D, F, L);
        pdflController.setTarget(liftTarget);
        pdflController.update(liftMotor.getCurrentPosition());



        power = pdflController.runPDFL(allowedError);
        liftMotor.setPower(power);
    }



    public void debugTelemetry(Telemetry telemetry){
        telemetry.addData("Power Supplied ", power);
        telemetry.addData("Motor Encoder ", liftMotor.getCurrentPosition());
        telemetry.addData("Target ", liftTarget);
    }





}


