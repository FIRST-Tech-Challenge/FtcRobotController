package org.firstinspires.ftc.teamcode.robots.catbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.*;

@Autonomous(name = "catbotAuton")
public class tonyLanehzAuton extends OpMode {
    //hardware variables
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackRight = null;
//    private DcMotorEx elevator = null;
//    private Servo claw = null;

    // regular drive
    private double powerLeft = 0;
    private double powerRight = 0;

    //number variables
    //private static final float DEADZONE = .1f;
    private static final int MAXELEVTICS = 4320;
    private static final int MINELEVTICS = 0;
    private int currElevTics = 0;
    private final double MOTORSTALLVALUE = .7;

    //boolean variables
    private boolean calibrate = true;


    @Override
    public void init() {
    motorInit();

    }

    @Override
    public void loop() {
    autonTankDrive();
    telemetry.addData("auton running", motorFrontRight.getCurrentPosition());
    telemetry.update();

    }

    public void autonTankDrive() {
        if(calibrate) {
            motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + 100);
            motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + 100);
            motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + 100);
            motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + 100);
        }
    }

    public void calib(){
//        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        telemetry.addData("elevator calibrating...", elevator.getCurrent(CurrentUnit.AMPS));
//        if(elevator.getCurrent(CurrentUnit.AMPS) < MOTORSTALLVALUE)
//        {
//            elevator.setPower(-.2);
//        }
//        else {
            calibrate = true;
//            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            elevator.setTargetPosition(0);
//            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            telemetry.addData("elevator position", elevator.getCurrentPosition());
//            telemetry.addData("done calibrating", elevator.getCurrentPosition());
        }
//    }
    public void motorInit(){
        motorFrontLeft = this.hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = this.hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = this.hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = this.hardwareMap.get(DcMotor.class, "motorBackRight");
//        elevator = this.hardwareMap.get(DcMotorEx.class, "elevator");
//        claw = this.hardwareMap.get(Servo.class, "claw");
//        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        this.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
//        this.elevator.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //this.motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        //this.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
