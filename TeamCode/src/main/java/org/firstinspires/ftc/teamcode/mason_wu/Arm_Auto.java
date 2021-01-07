package org.firstinspires.ftc.teamcode.mason_wu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

@Autonomous(name="Arm Autonomous", group="4100")
public class Arm_Auto extends LinearOpMode {
    // Declare OpMode members.

    private DcMotor Arm = null;
    private Servo Hand = null;

    private DcMotor Shooter = null;
    private Servo Spanker = null;

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;

    //other variables
    int cycle = 1;
    final double HAND_CLOSE_POSITION = 0.0;
    final double HAND_OPEN_POSITION = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables.

        //PLEASE MAKE SURE THAT THE deviceName IS THE SAME HERE TO THE NAMES IN THE CONFIGURATION MENU
        Hand = hardwareMap.get(Servo.class, "hand");
        Arm = hardwareMap.get(DcMotor.class, "arm");

        Spanker = hardwareMap.get(Servo.class, "spanker");
        Shooter = hardwareMap.get(DcMotor.class, "shooter");

        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        //PLEASE SET THE DIRECTION AND POSITION HERE

        Arm.setDirection(DcMotor.Direction.FORWARD);
//        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Hand.setPosition(HAND_CLOSE_POSITION);

        Shooter.setDirection(DcMotor.Direction.REVERSE);
        Spanker.setPosition(0.4);

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        waitForStart();

        while (opModeIsActive()) {
             if(cycle == 1) {
                 //arm moves up
                 armMotion(true, 0.3, 800);
                 sleep(200);
                 //arm moves down
                 armMotion(false, 0.3, 800);
             }
             //handMotion(true);
            telemetry.addData("Hand Position:", Hand.getPosition());
            telemetry.update();
            cycle++;

        }
    }

    boolean armMotion(boolean moveUp, double power, double timeInterval){
        ElapsedTime armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        int Upfactor = -1;
        if(!moveUp){
            Upfactor = 1;
        }
        while(armTime.milliseconds() <= timeInterval) {
            Arm.setPower(Math.abs(power)*(Upfactor));
        }
        return true;
    }

    boolean handMotion(boolean close){
        if(close) {
            Hand.setPosition(HAND_CLOSE_POSITION);
        }
        else{
            Hand.setPosition(HAND_OPEN_POSITION);
        }
        return true;
    }
}