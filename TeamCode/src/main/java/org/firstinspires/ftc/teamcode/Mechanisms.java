package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Mechanisms {

    ElapsedTime totalTime = new ElapsedTime();

    // outake lift right
    DcMotor outTakeLiftRight;
    // outake lift left
    DcMotor outTakeLiftLeft;

    // outtake servos
    Servo outTakePivotRight;
    Servo outTakePivotLeft;
    Servo outTakeClaw;
    Servo outTakeFlip;

    // intake lifts
    DcMotor inTakeLift;

    // intake servos
    Servo inTakeClaw;
    Servo intakePivotR;
    Servo intakePivotL;
    Servo inTakeRotator;



    // Mechanism stuff
    double CLOSED_OT_POS = 0;
    double OPEN_OT_POS = .5;
    double clawInc = 0;
    double lastClawTime;
    double UP_OT_FLIP_POS = 0;
    double DOWN_OT_FLIP_POS = 1;
    double UP_OT_PIVOT_POS = 1;
    double DOWN_OT_PIVOT_POS = 0;

    double PERP_IT_POS = 0;
    double PAR_IT_POS = .33;
    double CLOSED_IT_POS = 1;
    double OPEN_IT_POS = .76;
    double UP_IT_FLIP_POS = .7;
    double DOWN_IT_FLIP_POS = .42;
    double MID_IT_FLIP_POS = .5;
    double OUT_IT_FLIP_POS = 0;
    double OUT_DOWN_IT_FLIP_POS = .26;

    // backright drivetrain motor port 0 control
    // backleft drivetrain motor port 1 control
    // frontleft drivetrain motor port 1 expansion
    // frontright drivetrain motor port 0 expansion
    // outtake lift right motor port 2 control
    // intakeflipleft servo 1 control
    // intake claw servo 2 control
    // intake rotator servo 3 control



    boolean upPivotOT = true;
    double pivotTimeOT = 0;

    OpMode master;

    public void init(OpMode opMode)
    {
        // outtake lifts
        outTakeLiftRight = opMode.hardwareMap.dcMotor.get("otlr");
        outTakeLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outTakeLiftLeft = opMode.hardwareMap.dcMotor.get("otll");
        outTakeLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // outtake servos
        outTakeClaw = opMode.hardwareMap.servo.get(("otc"));
        outTakeFlip = opMode.hardwareMap.servo.get(("otf"));
        outTakePivotRight = opMode.hardwareMap.servo.get("otpr");
        outTakePivotLeft = opMode.hardwareMap.servo.get("otpl");


        // Intake lift
        //inTakeLift = opMode.hardwareMap.dcMotor.get("itl");
        //inTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //inTakeFlipControl.getController().pwmEnable();

        // intake servos
        intakePivotL = opMode.hardwareMap.servo.get(("itpl")); // port
        intakePivotR = opMode.hardwareMap.servo.get(("itpr")); // port
        inTakeRotator = opMode.hardwareMap.servo.get("itr"); // port
        inTakeClaw = opMode.hardwareMap.servo.get(("itc")); // port


        // Expansion hub servos
        //outTakeLargePivotExpansion = opMode.hardwareMap.servo.get(("otse"));
        //outTakeLargePivotControl = opMode.hardwareMap.servo.get(("otsc"));
        //outTakeClawPivot = opMode.hardwareMap.servo.get(("otcp"));
        //outTakeClaw = opMode.hardwareMap.servo.get(("otc"));

        master = opMode;
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setOutTakeLift(){
        outTakeLiftRight.setPower(master.gamepad2.left_stick_y);
        outTakeLiftLeft.setPower(master.gamepad2.left_stick_y);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setOutTakeClawGrab(){
        if (master.gamepad2.x && lastClawTime < totalTime.milliseconds() - 500)
        {
            lastClawTime = totalTime.milliseconds();
            clawInc += .1;
            outTakeClaw.setPosition(clawInc);
            outTakeClaw.setDirection(Servo.Direction.REVERSE);
        }
        //outTakeClaw.setPosition(CLOSED_OT_POS);
        if (master.gamepad2.y && lastClawTime < totalTime.milliseconds() - 500)
        {
            lastClawTime = totalTime.milliseconds();
            clawInc -= .1;
            outTakeClaw.setPosition(clawInc);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setOutTakeFlip(){
        if (master.gamepad2.left_trigger > .1)
            outTakePivotRight.setPosition(DOWN_OT_FLIP_POS);
        if (master.gamepad2.right_trigger > .1)
            outTakePivotLeft.setPosition(UP_OT_FLIP_POS);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setOutTakePivot(){
        if (master.gamepad2.dpad_right && pivotTimeOT < totalTime.milliseconds() - 500)
        {
            if (upPivotOT)
            {
                outTakeFlip.setPosition(DOWN_OT_PIVOT_POS);
                upPivotOT = false;
            }
            else
            {
                outTakeFlip.setPosition(UP_OT_PIVOT_POS);
                upPivotOT = true;
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setInTakeClawGrab(){
        if (master.gamepad2.a)
            inTakeClaw.setPosition(CLOSED_IT_POS);
        if (master.gamepad2.b)
            inTakeClaw.setPosition(OPEN_IT_POS);
        if (master.gamepad2.dpad_right)
            inTakeClaw.setPosition(.65);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setInTakeFlip(){
        if (master.gamepad2.dpad_up)
            intakePivotL.setPosition(UP_IT_FLIP_POS);
            intakePivotR.setPosition(UP_IT_FLIP_POS);

        if (master.gamepad2.dpad_down)
            intakePivotR.setPosition(DOWN_IT_FLIP_POS);
            intakePivotL.setPosition(DOWN_IT_FLIP_POS);
        if (master.gamepad2.dpad_left)
        {
            intakePivotR.setPosition(MID_IT_FLIP_POS);
            intakePivotL.setPosition(MID_IT_FLIP_POS);
        }
        if (master.gamepad2.x)
            intakePivotR.setPosition(OUT_IT_FLIP_POS);
            intakePivotL.setPosition(OUT_IT_FLIP_POS);
        if (master.gamepad2.y)
            intakePivotR.setPosition(OUT_DOWN_IT_FLIP_POS);
            intakePivotL.setPosition(OUT_DOWN_IT_FLIP_POS);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setInTakeRotator(){
        if (master.gamepad2.left_bumper)
            inTakeRotator.setPosition(PAR_IT_POS);
        if (master.gamepad2.right_bumper)
            inTakeRotator.setPosition(PERP_IT_POS);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setInTakeLift(){
        inTakeLift.setPower(master.gamepad2.right_stick_y);
        if (master.gamepad2.left_trigger > .1)
        {
            inTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (master.gamepad2.right_trigger > .1)
        {
            inTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    //////////////////////////////////////////////////////////////////////////////
    public void transfer() throws InterruptedException{
        intakePivotL.setPosition(1); // pos of intake so that it is lifted
        intakePivotR.setPosition(1); // pos of intake so that it is lifted
        master.wait(200);
        // move intake to transfer here
        master.wait(2);
        intakePivotL.setPosition(0); // pos of intake when it is in the transfer position
        intakePivotR.setPosition(0); // pos of intake when it is in the transfer position
        master.wait(200);
        outTakeFlip.setPosition(0); // we move the outtake claw so that it is in a position where it can move past the slides
        outTakePivotRight.setPosition(0); // pos of outtake when it is in the transfer position
        outTakePivotLeft.setPosition(0); // pos of outtake when it is in the transfer position
        master.wait(200);
        outTakePivotRight.setPosition(0); // pivot outtake so that it is in the pos where it drops the pixel
        outTakePivotLeft.setPosition(0); // pivot outtake so that it is in the pos where it drops the pixel
    }
    //////////////////////////////////////////////////////////////////////////////
    public void slidePosHigh() {
        outTakeLiftLeft.setTargetPosition(1); // encoder position of highest position slides need to be
        outTakeLiftRight.setTargetPosition(1);

    }
    //////////////////////////////////////////////////////////////////////////////
    public void servotesting() {
        double rotatorConstant = 0.0;
        if (master.gamepad1.x == true) {
            rotatorConstant += 0.1;
            intakePivotL.setPosition(rotatorConstant);
            intakePivotR.setPosition(rotatorConstant);
            master.telemetry.update();
            master.telemetry.addData("Servo position", rotatorConstant);
        }

    }

    //////////////////////////////////////////////////////////////////////////////
    public void runTesting()
    {
        if (master.gamepad2.a)
        {
            // up
            inTakeClaw.setPosition(.76);
        }
        if (master.gamepad2.b)
        {
            //grab
            intakePivotL.setPosition(.3);
            intakePivotR.setPosition(.3);
            //inTakeFlipControl.getController().setServoPosition();
        }
        if (master.gamepad2.dpad_up)
        {
            //put in transfer
            intakePivotL.setPosition(.5);
            intakePivotR.setPosition(.5);
        }
        if (master.gamepad2.dpad_down)
        {
            //put in transfer
            inTakeRotator.setPosition(.32);
        }
        if (master.gamepad2.dpad_left)
        {
            inTakeRotator.setPosition(0);
        }
        if (master.gamepad2.x)
        {
            intakePivotL.setPosition(.7);
            intakePivotR.setPosition(.7);
        }
        if (master.gamepad2.y)
        {
            inTakeClaw.setPosition(1);
        }
        if (master.gamepad2.left_bumper)
        {
            outTakeLiftLeft.setPower(.8);
            outTakeLiftRight.setPower(.8);
        }
        else if (master.gamepad2.right_bumper)
        {
            outTakeLiftLeft.setPower(-.8);
            outTakeLiftRight.setPower(-.8);
        }
        else
        {
            outTakeLiftLeft.setPower(0);
            outTakeLiftRight.setPower(0);
        }
        if (master.gamepad2.left_trigger > .1)
        {
            inTakeLift.setPower(.8);
        }
        else if (master.gamepad2.right_trigger > .1)
        {
            inTakeLift.setPower(-.8);
        }
        else {
            inTakeLift.setPower(0);
        }
    }

}
