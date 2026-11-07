
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {
   // public Limelight3A limelight;
    public DcMotorEx shooterMotor;
   // public DcMotor shooterIntakeMotor;

    private OpMode myOpMode;

    public double power = 200 ;
   private DcMotorEx intakeMotor;
    private DcMotor transfer = null;
    private DcMotor transfer2 = null;


    // ---- INITS ----



    public RobotHardware(OpMode opmode) {
        myOpMode = opmode;
        init();
    }

    private void init() {
        initMotors();
        //initLimelight();
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }



    // ---------------- MOTORS ----------------
    private void initMotors() {
        intakeMotor   = myOpMode.hardwareMap.get(DcMotorEx.class, "inTakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer = myOpMode.hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer2 = myOpMode.hardwareMap.get(DcMotor.class, "transfer2");
        transfer2.setDirection(DcMotor.Direction.REVERSE);
       // shooterIntakeMotor = myOpMode.hardwareMap.get(DcMotor.class, "shooterIntakeMotor");
       // shooterIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void intakeOn() {
        intakeMotor.setVelocity(power, AngleUnit.DEGREES);
    }

    public void transferOn(){
        transfer.setPower(-1);
        transfer2.setPower(1);

    }
    public void transferOn2(){
        transfer2.setPower(0.5);
    }
    // public void shooterIntakeOn(){
   //     shooterIntakeMotor.setPower(1);
   // }
   // public void shooterIntakeOff(){
    //    shooterIntakeMotor.setPower(0);
   // }
    public void outtakeOn(){
        intakeMotor.setPower(-0.3);
        transfer.setPower(0.5);
        transfer2.setPower(-0.5);

    }
    // public void outtakeTrasferOn(){transfer.setPower(-1);}
    // public void outtakeTransferOn2(){transfer2.setPower(-1);}
    public void intakeOff() {
        intakeMotor.setPower(0);
        transfer.setPower(0);
        transfer2.setPower(0);
    }
    public void shooterON(){shooterMotor.setPower(0.75);}
    public void shooterOFF(){shooterMotor.setPower(0);}
    public void intakeSpeedUp(){power = power + 10;}
    public void intakeSpeedDown(){power = power - 10;}


    // ---------------- LIMELIGHT ----------------
   // private void initLimelight() {
        //limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight");
  //  }






    // ---------------- BELTS ----------------


    // ---------------- SHOOTER ----------------

    // ---------------- BLANK ------------------

}


