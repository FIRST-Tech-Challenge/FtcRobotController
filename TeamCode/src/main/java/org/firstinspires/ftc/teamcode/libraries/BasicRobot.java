package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.libraries.servo.AjustableServo;
import org.firstinspires.ftc.teamcode.libraries.servo.FixedPositionServo;
//where all the extra motors and servos are set
public interface BasicRobot {

    // first value is the transfer position and other is outside position
    int TRANSFER = 0;
    int GRAB = 1;
    int PLACE = 1;
    FixedPositionServo outtakeAngle = new FixedPositionServo("outtakeAngle", new double[]{0.4375, 0.58});
    FixedPositionServo intakeAngle = new FixedPositionServo("intakeAngle", new double[]{0.75, 0.06});

    //first position is out and second is all the way in
    int SLIDEOUT = 0;
    int SLIDEIN = 1;
    AjustableServo intakeSlide1 = new AjustableServo("intakeSlide1", 0,0,0.35,0.0025);
    AjustableServo intakeSlide2 = new AjustableServo("intakeSlide2", 1, 0.65, 1, 0.0025);

    int OPEN = 0;
    int CLOSE = 1;
    int SLIGHTCLOSE = 2;
    FixedPositionServo outtakeClaw = new FixedPositionServo("outtakeClaw", new double[]{0.036, 0.3});
    FixedPositionServo intakeClaw = new FixedPositionServo("intakeClaw", new double[]{0.0D, 0.05, 0.46});

    Motor elavator1 = new Motor("elavator1", true, true);
    Motor elavator2 = new Motor("elavator2", false, true);

    int HIGH_BASKET = 3300;
    //loads the extra motors and servos for this season
    default void loadAll(HardwareMap hardwareMap){
        //motors
        elavator1.setMotor(hardwareMap.get(DcMotor.class, elavator1.motorname));
        elavator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elavator1.setupMotor();
        elavator2.setMotor(hardwareMap.get(DcMotor.class, elavator2.motorname));
        elavator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elavator2.setupMotor();

        //servos
        outtakeAngle.setServo(hardwareMap.get(Servo.class, outtakeAngle.servoName));
        intakeAngle.setServo(hardwareMap.get(Servo.class, intakeAngle.servoName));
        intakeSlide1.setServo(hardwareMap.get(Servo.class, intakeSlide1.servoName));
        intakeSlide2.setServo(hardwareMap.get(Servo.class, intakeSlide2.servoName));
        outtakeClaw.setServo(hardwareMap.get(Servo.class, outtakeClaw.servoName));
        intakeClaw.setServo(hardwareMap.get(Servo.class, intakeClaw.servoName));
    }

    default void elevator(int ticks){
        elavator1.move(ticks);
        elavator2.move(ticks);
    }
    default void elevatorWhileMove(){
        if (elavator1.getMotor().isBusy()){
            elavator2.setPower(0.7);
            elavator1.setPower(0.7);
        }
        if(!(elavator1.getMotor().isBusy())){
            elavator2.setPower(0.1);
            elavator1.setPower(0.1);
        }
    }

    default void elevatorMove(int ticks){
        elavator1.move(ticks);
        elavator2.move(ticks);
    }

    default void elevatorWait(){
        while (elavator1.getMotor().isBusy()){

        }
    }
}


