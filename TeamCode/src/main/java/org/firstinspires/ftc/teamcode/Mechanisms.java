package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Config
public class Mechanisms {

    ElapsedTime totalTime = new ElapsedTime();
    ElapsedTime outtakeTransferPos = new ElapsedTime();

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
    DcMotor inTakeSpinners;

    // intake servos
    Servo intakePivotR;
    Servo intakePivotL;

    // color sensor in intake
    ColorSensor intakeColorSensor;

    // color sensor values
    double redValue;
    double blueValue;
    double greenValue;
    double alphaValue; // light intensity
    double tarVal = 1000;

    // Mechanism stuff
    public double lastClawTime;
    public static double PLACE_OT_FLIP_POS = .35;
    public static double UP_OT_FLIP_POS = .25;
    public static double MID_OT_FLIP_POS = .15;
    public static double DOWN_OT_FLIP_POS = 0;
    public static double BUCKET_OT_PIVOT_POS = .5;
    public static double TRANSFER_OT_PIVOT_POS = .67;
    public static double MIDDLE_OT_PIV_POS = .5; // still needs to be tuned
    public static double PICKUP_OT_PIVOT_POS = .23;

    public static double OT_CLAW_GRAB = .68;
    public static double OT_CLAW_RELEASE = 1;

    public static double PERP_IT_POS = .82;
    public static double PAR_IT_POS = .33;
    public static double CLOSED_IT_POS = 1;
    public static double OPEN_IT_POS = .76;
    public static double MIDDLE_IT_POS = .88;
    public static double UP_IT_FLIP_POS = 1;
    public static double DOWN_IT_FLIP_POS = .29;
    public static double MID_IT_FLIP_POS = .5;

    ElapsedTime intakeToTransfer = new ElapsedTime();

    String ITMacroState = "none";
    ElapsedTime inTakeAndUpStateTime = new ElapsedTime();

    String TransferMacroState = "none";
    ElapsedTime TransferMacroStateTime = new ElapsedTime();

    boolean OTGrabbed = false;
    boolean ITGrabbed = false;
    ElapsedTime OT_ARM_TO_NEUTRAL_POS_TIME = new ElapsedTime();

    double brakePosIT = 0;
    boolean brakingIT;

    double brakePosOT = 0;
    boolean brakingOT = true;
    // backright drivetrain motor port 0 control
    // backleft drivetrain motor port 1 control
    // frontleft drivetrain motor port 1 expansion
    // frontright drivetrain motor port 0 expansion
    // outtake lift right motor port 2 control
    // intakeflipleft servo 1 control
    // intake claw servo 2 control
    // intake rotator servo 3 control

    // to build a custom rumble effect
    Gamepad.RumbleEffect customRumbleEffect;


    public static boolean upPivotOT = true;
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

        inTakeLift = opMode.hardwareMap.dcMotor.get("itl");
        inTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outTakeLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outTakeLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*// color sensor
        intakeColorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");*/

        //Intake lift
        inTakeLift = opMode.hardwareMap.dcMotor.get("itl");
        inTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Intake spinners
        inTakeSpinners = opMode.hardwareMap.dcMotor.get("its");
        inTakeSpinners.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //inTakeFlipControl.getController().pwmEnable();

        // intake servos
        intakePivotL = opMode.hardwareMap.servo.get(("itpl"));
        intakePivotR = opMode.hardwareMap.servo.get(("itpr"));

        brakePosIT = inTakeLift.getCurrentPosition();
        outTakeLiftRight = opMode.hardwareMap.dcMotor.get("otlr");
        outTakeLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outTakeLiftLeft = opMode.hardwareMap.dcMotor.get("otll");
        outTakeLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // outtake servos
        outTakeClaw = opMode.hardwareMap.servo.get(("otc"));
        outTakeFlip = opMode.hardwareMap.servo.get(("otf"));
        outTakePivotRight = opMode.hardwareMap.servo.get("otpr");
        outTakePivotLeft = opMode.hardwareMap.servo.get("otpl");

        inTakeLift = opMode.hardwareMap.dcMotor.get("itl");
        inTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outTakeLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outTakeLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //Intake lift
        inTakeLift = opMode.hardwareMap.dcMotor.get("itl");
        inTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Intake spinners
        inTakeSpinners = opMode.hardwareMap.dcMotor.get("its");
        inTakeSpinners.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //inTakeFlipControl.getController().pwmEnable();

        // intake servos
        intakePivotL = opMode.hardwareMap.servo.get(("itpl"));
        intakePivotR = opMode.hardwareMap.servo.get(("itpr"));

        brakePosIT = inTakeLift.getCurrentPosition();
        brakePosOT = (outTakeLiftLeft.getCurrentPosition() + outTakeLiftLeft.getCurrentPosition()) / 2.0;

        intakePivotL.setPosition(UP_IT_FLIP_POS - .15);

        brakePosOT = (outTakeLiftLeft.getCurrentPosition() + outTakeLiftLeft.getCurrentPosition()) / 2.0;

        intakePivotL.setPosition(UP_IT_FLIP_POS - .15);

        master = opMode;
    }


    ////////////////////////////////////////////////////////////////////////////////
    public void setBaseOuttakeLift()
    {
        outTakeLiftLeft.setPower(master.gamepad2.left_stick_y);
        outTakeLiftRight.setPower(master.gamepad2.left_stick_y);
    }
    ////////////////////////////////////////////////////////////////////////////////
    public void setBaseIntakeLift()
    {
        inTakeLift.setPower(master.gamepad2.right_stick_y);
    }
    ////////////////////////////////////////////////////////////////////////////////
    public void setOuttakeArmToNeutralPos()
    {
        if (master.gamepad2.b)
        {
            if (OT_ARM_TO_NEUTRAL_POS_TIME.milliseconds() > 200)
            {
                outTakeFlip.setPosition(DOWN_OT_FLIP_POS); // has to be tuned, the down position to flip the outtake to in order to allow it to pass through the slides
                OT_ARM_TO_NEUTRAL_POS_TIME.reset();
            }
            if (OT_ARM_TO_NEUTRAL_POS_TIME.milliseconds() > 200)
            {
                outTakePivotLeft.setPosition(MIDDLE_OT_PIV_POS); // has to be tuned, positions the outtake arm so that it is in between the slides but not in the intake position
                OT_ARM_TO_NEUTRAL_POS_TIME.reset();
            }

        }
    }
    ////////////////////////////////////////////////////////////////////////////////
    /*public void rumbleControllerWhenCorrectPieceIntook()
    {
        while(true)
        {
            intakeColorSensor.
            master.telemetry.addData()
        }
    }*/
    ////////////////////////////////////////////////////////////////////////////////
    public void setOutTakeLift(){

        double mult = Math.max(1 - master.gamepad2.left_trigger, .4);
        //outTakeLiftLeft.setPower(master.gamepad2.left_stick_y * mult);
        //outTakeLiftRight.setPower(master.gamepad2.left_stick_y * mult);
        if (Math.abs(master.gamepad2.left_stick_y) < .05 && !brakingOT)
        {
            brakingOT = true;
            brakePosOT = outTakeLiftLeft.getCurrentPosition();
        }
        else if (Math.abs(master.gamepad2.left_stick_y) > .05)
        {
            master.telemetry.addData("stick y", Math.abs(master.gamepad2.left_stick_y));
            brakingOT = false;
            outTakeLiftLeft.setPower(master.gamepad2.left_stick_y * mult);
            outTakeLiftRight.setPower(master.gamepad2.left_stick_y * mult);
        }
        if (brakingOT)
        {
            master.telemetry.addData("diff", outTakeLiftLeft.getCurrentPosition() - brakePosOT);
            outTakeLiftLeft.setPower((outTakeLiftLeft.getCurrentPosition() - brakePosOT) / 100 * .07);
            outTakeLiftRight.setPower((outTakeLiftLeft.getCurrentPosition() - brakePosOT) / 100 * .07);
        }
    }

    public void moveOTLiftEncoder(double power, double tarPos, double timeOut)
    {
        ElapsedTime time = new ElapsedTime();
        double pos = brakePosOT = (outTakeLiftLeft.getCurrentPosition() + outTakeLiftLeft.getCurrentPosition()) / 2.0;
        while (Math.abs(Math.abs(tarPos) - Math.abs(pos)) > 20 && time.milliseconds() < timeOut)
        {
            double sign = Math.signum(tarPos - pos);
            outTakeLiftLeft.setPower(Math.min(power, Math.pow(tarPos - pos, 2) / 1000 * power * sign));
        }

    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setOutTakeClawGrab(){
        if (master.gamepad2.y && lastClawTime < totalTime.milliseconds() - 500)
        {
            master.telemetry.addData("tt", totalTime.milliseconds());
            if (!OTGrabbed)
            {
                lastClawTime = totalTime.milliseconds();
                outTakeClaw.setPosition(OT_CLAW_GRAB);
                OTGrabbed = true;
            }
            else
            {
                lastClawTime = totalTime.milliseconds();
                outTakeClaw.setPosition(OT_CLAW_RELEASE);
                OTGrabbed = false;
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////

    public void setOutTakeFlip(){
        if (master.gamepad2.right_trigger > .1)
            outTakeFlip.setPosition(UP_OT_FLIP_POS);
        if (master.gamepad2.b)
            outTakeFlip.setPosition(MID_OT_FLIP_POS);
        if (master.gamepad2.back)
            outTakeFlip.setPosition(UP_OT_FLIP_POS + .15);
    }

    public void transferMacro()
    {
        outTakeClaw.setPosition(OT_CLAW_GRAB); // set the outtake claw to its most closed position
        if (master.gamepad2.a && TransferMacroStateTime.milliseconds() > 200)
        {
            TransferMacroStateTime.reset();
            if (TransferMacroStateTime.milliseconds() > 100)
            {
                intakePivotL.setPosition(UP_IT_FLIP_POS); // will have to tune, position the intake in the middle of the drivetrain
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakeFlip.setPosition(DOWN_OT_FLIP_POS); // will have to tune, meant to be a position to get the outtake flipped to the lowest possible position (arm ready to be moved down)
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakePivotLeft.setPosition(DOWN_OT_FLIP_POS); // moves whole outtake arm downwards to the position to be transferred
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakeFlip.setPosition(DOWN_OT_FLIP_POS); // will have to tune, meant to be a position to get the outtake flipped to the position so that it is ready to grab the sample
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakeClaw.setPosition(OT_CLAW_GRAB); // will have to tune, meant to be a middle position to fit inside the intake but still grab the sample inside the intake
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakeFlip.setPosition(DOWN_OT_FLIP_POS); // will have to tune, meant to move the outtake arm down to get the claw in position to clamp and grab the sample
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakeClaw.setPosition(OT_CLAW_GRAB); // will have to tune, clamps the sample in the intake
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakePivotLeft.setPosition(UP_OT_FLIP_POS); // will have to tune, meant to move the outtake arm up to move the claw to the position to outtake, still needs to be flipped
            }
            TransferMacroStateTime.reset();
            if (intakeToTransfer.milliseconds() > 200)
            {
                outTakeFlip.setPosition(UP_OT_FLIP_POS); // will have to tune, meant to flip the outtake to the highest position to be able to drop the sample
            }
            TransferMacroStateTime.reset();
        }
    }

    /*public void outTakeMacroAndTransfer()
    {
        if (master.gamepad2.a && outTakeAndUpStateTime.milliseconds() > 500 && OTMacroState.equals("none"))
        {
            outTakeAndUpStateTime.reset();
            OTMacroState = "clamp";
        }
        if (OTMacroState.equals("clamp"))
        {
            outTakeClaw.setPosition(OT_CLAW_GRAB);
            if (outTakeAndUpStateTime.milliseconds() > 200) {
                inTakeClaw.setPosition(OPEN_IT_POS);
                if (outTakeAndUpStateTime.milliseconds() > 300) {
                    outTakeAndUpStateTime.reset();
                    OTMacroState = "raise";
                }
            }
        }
        if (OTMacroState.equals("raise")) {
            outTakePivotRight.setPosition(BUCKET_OT_PIVOT_POS);
            outTakeFlip.setPosition(PLACE_OT_FLIP_POS);
            if (outTakeAndUpStateTime.milliseconds() > 750) {
                outTakeAndUpStateTime.reset();
                OTMacroState = "none";
            }
        }
    }*/


    ////////////////////////////////////////////////////////////////////////////////
    public void setOutTakePivot(){
        if (master.gamepad2.dpad_right && pivotTimeOT < totalTime.milliseconds() - 500)
        {
            if (upPivotOT)
            {
                outTakePivotRight.setPosition(TRANSFER_OT_PIVOT_POS);
                outTakeClaw.setPosition(OT_CLAW_GRAB);
                upPivotOT = false;
            }
            else
            {
                outTakePivotRight.setPosition(BUCKET_OT_PIVOT_POS);
                outTakeClaw.setPosition(OT_CLAW_GRAB);
                outTakeFlip.setPosition(PLACE_OT_FLIP_POS);
                upPivotOT = true;
            }
            pivotTimeOT = totalTime.milliseconds();
        }
        if (master.gamepad2.x && pivotTimeOT < totalTime.milliseconds() - 500)
        {
            //outTakeClaw.setPosition(OT_CLAW_RELEASE);
            outTakeFlip.setPosition(MID_OT_FLIP_POS);
            outTakePivotRight.setPosition(.2);
            pivotTimeOT = totalTime.milliseconds();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    /*public void setInTakeClawGrab(){
        if (master.gamepad2.b && ITGrabTime.milliseconds() > 500) {
            if (ITGrabbed) {
                inTakeClaw.setPosition(OPEN_IT_POS);
                ITGrabbed = false;
            }
            else {
                inTakeClaw.setPosition(CLOSED_IT_POS);
                ITGrabbed = true;
            }
            ITGrabTime.reset();
        }
        if (master.gamepad2.dpad_right)
            inTakeClaw.setPosition(MIDDLE_IT_POS);
    }*/

    ////////////////////////////////////////////////////////////////////////////////
    /*public void setInTakeFlip(){
        if (master.gamepad2.dpad_up) {
            intakePivotL.setPosition(UP_IT_FLIP_POS);
           // intakePivotR.setPosition(UP_IT_FLIP_POS);
        }

        if (master.gamepad2.dpad_down) {
          //  intakePivotR.setPosition(DOWN_IT_FLIP_POS);
            intakePivotL.setPosition(DOWN_IT_FLIP_POS);
        }
        if (master.gamepad2.dpad_left)
        {
          //  intakePivotR.setPosition(MID_IT_FLIP_POS);
            intakePivotL.setPosition(MID_IT_FLIP_POS);
        }
    }*/

    ////////////////////////////////////////////////////////////////////////////////
    /*public void setInTakeRotator(){
        if (master.gamepad2.left_bumper)
            inTakeRotator.setPosition(PAR_IT_POS);
        if (master.gamepad2.right_bumper)
            inTakeRotator.setPosition(PERP_IT_POS);
    }*/

    ////////////////////////////////////////////////////////////////////////////////
    public void setInTakeLift(){
        inTakeLift.setPower(master.gamepad2.right_stick_y * .4);
        if (Math.abs(master.gamepad2.right_stick_y) < .05 && !brakingIT)
        {
            brakingIT = true;
            brakePosIT = inTakeLift.getCurrentPosition();
        }
        else if (Math.abs(master.gamepad2.right_stick_y) > .05)
        {
            brakingIT = false;
        }
        if (brakingIT)
        {
          //  inTakeLift.setPower((inTakeLift.getCurrentPosition() - brakePosIT) / 100 * .05);
        }
    }
    //////////////////////////////////////////////////////////////////////////////
    public void setIntakeSpinners()
    {
        if (master.gamepad2.right_trigger > 0.4)
        {
            inTakeSpinners.setPower(1);
        }
        if (master.gamepad2.left_trigger > 0.4)
        {
            inTakeSpinners.setPower(-1);
        }
    //////////////////////////////////////////////////////////////////////////////
    /*public void transfer() throws InterruptedException{
        if (intakeToTransfer.milliseconds() > 0 && intakeToTransfer.milliseconds() < 200)
        {
            intakePivotL.setPosition(0);
            intakePivotR.setPosition(0);
        }
        if (intakeToTransfer.milliseconds() > 200 && intakeToTransfer.milliseconds() < 400)
        {
            intakePivotL.setPosition(1); // pos of intake so that it is lifted
            intakePivotR.setPosition(1);
        }
        if (intakeToTransfer.milliseconds() > 400 && intakeToTransfer.milliseconds() < 600)
        {
            intakePivotL.setPosition(0); // pos of intake when it is in the transfer position
            intakePivotR.setPosition(0);
        }
        if (intakeToTransfer.milliseconds() > 600 && intakeToTransfer.milliseconds() < 800)
        {
            outTakeFlip.setPosition(0);
            intakePivotL.setPosition(0); // pos of intake so that it is lifted
            intakePivotR.setPosition(0);
        }
        if (intakeToTransfer.milliseconds() > 800 && intakeToTransfer.milliseconds() < 1000)
        {
            outTakePivotRight.setPosition(0); // pivot outtake so that it is in the pos where it drops the pixel
            outTakePivotLeft.setPosition(0);
        }
    }*/
    //////////////////////////////////////////////////////////////////////////////
    /*public void slidePosHigh() {
        outTakeLiftLeft.setTargetPosition(1); // encoder position of highest position slides need to be
        outTakeLiftRight.setTargetPosition(1);

    }*/
    /*
    //////////////////////////////////////////////////////////////////////////////
    public void servotesting() {
        double rotatorConstant = 0.0;
        if (master.gamepad1.x) {
            rotatorConstant += 0.1;
            intakePivotL.setPosition(rotatorConstant);
            intakePivotR.setPosition(rotatorConstant);
            master.telemetry.update();
            master.telemetry.addData("Servo position", rotatorConstant);
        }

    }

     */

    //////////////////////////////////////////////////////////////////////////////
    /*public void runTesting()
    {
        /*if (master.gamepad2.a)
        {
            // up
            inTakeClaw.setPosition(.76);
        }*/
        /*if (master.gamepad2.b)
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
        }*/
        /*if (master.gamepad2.dpad_down)
        {
            //put in transfer
            inTakeRotator.setPosition(.32);
        }*/
        /*if (master.gamepad2.dpad_left)
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
        }*/
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
