package org.firstinspires.ftc.teamcode;
import androidx.annotation.VisibleForTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
@Disabled
public class MainAuto extends LinearOpMode {
    ///////////////////////////////psuedocode///////////////////////////////
    //(robot is 17 inches long)
    //move right 24 inches
    //extend linear slide up to second ladder
    //move forward 7 inches(24-17)
    //move slide down slightly (clip specimen on second ladder)
    //release claw
    //move left a few inches (for other team's auto

    ///////////////////////////////code///////////////////////////////
    private ElapsedTime     runtime = new ElapsedTime();
    static final double countsPerRev = 1440;
    static final double wheelDiameter = 3.5;     // For figuring circumference (in inches)
    static final double countsPerInch  = countsPerRev / (wheelDiameter * 3.141592653589793238462643383279502884197169399375);
    private DcMotor leftBack; //Initializes Back-Left direct current motor for the driving function of our robot, gary.
    private DcMotor rightBack; //Initializes Back-Right direct current motor for the driving function of our robot, gary.
    private DcMotor leftFront; //Initializes Front-Left direct current motor for the driving function of our robot, gary.
    private DcMotor rightFront; //Initializes Front-Right direct current motor for the driving function of our robot, gary.
    private Servo clawServo;

    @Override
    public void runOpMode() {
        leftBack  = hardwareMap.get(DcMotor.class, "bl");
        rightBack  = hardwareMap.get(DcMotor.class, "br");
        leftFront  = hardwareMap.get(DcMotor.class, "fl");
        rightFront  = hardwareMap.get(DcMotor.class, "fr");

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        telemetry.addData("Starting pos: ", leftBack.getCurrentPosition());
        telemetry.update();
        waitForStart();

        driveInches(24, 1, dir.RIGHT, 5);
        //extend linear slide up to second ladder
        driveInches(7, 1, dir.FORWARD, 5);
        //move slide down slightly (clip specimen on second ladder)
        //release claw
        //move slide up slightly
        driveInches(3, 1, dir.BACKWARD, 5);
        driveInches(30, 1, dir.LEFT, 5);
        driveInches(24, 1, dir.FORWARD, 5);

    }
    enum dir {
        LEFT,
        RIGHT,
        FORWARD,
        BACKWARD

    }
    private void driveInches(float inches, float speed, dir direction, float timeoutS) {
        int lbDir = 1;
        int rbDir = 1;
        int lfDir = 1;
        int rfDir = 1;
        int targetPos;

        //sets some motors to negative power depending on direction
        switch(direction) {
            case LEFT:
                rbDir = -1;
                lfDir = -1;
                break;
            case RIGHT:
                lbDir = -1;
                rfDir = -1;
                break;
            case FORWARD:
                break;
            case BACKWARD:
                lbDir = -1;
                rbDir = -1;
                lfDir = -1;
                rfDir = -1;
                break;
        }
        if(opModeIsActive()) {
            runtime.reset();
            targetPos = lbDir + (int)(inches * countsPerInch);
            leftBack.setPower(lbDir * speed);
            rightBack.setPower(rbDir * speed);
            leftFront.setPower(lfDir * speed);
            rightFront.setPower(rfDir * speed);
            while(opModeIsActive() && timeoutS < runtime.seconds()) {
                telemetry.addData("currently going", String.valueOf(direction), " to ", targetPos);
                telemetry.update();
            }
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            sleep(100);
        }
    }

    void moveClaw(boolean open) {
        if(open) {
            clawServo.setPosition(0);
        } else {
            clawServo.setPosition(1);
        }
    }
}
