package org.firstinspires.ftc.teamcode;// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

@Autonomous(name="FTC 14133 2021 Auto", group="Auto")
public class FTC_14133_2021_Auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx lb = null;        // Sets the variables of the mecanum wheels
    private DcMotorEx rb = null;
    private DcMotorEx lf = null;
    private DcMotorEx rf = null;
    static final double MOTOR_TICK_COUNT = 2800;        //
    private DcMotorEx shooter = null;         // Sets the variable of the shooter
    private DcMotorEx arm = null;         // Sets the variable of the arm that is long but there is not a arm that is short
    private DcMotorEx intake = null;          // Sets the variable of the intake
    private DcMotorEx conveyor = null;          // Sets the variable of the conveyor
    //   DigitalChannel LimitSwitchLongArm;          // Sets the variable of the LimitSwitchLongArm
    DigitalChannel beambreak;          // Sets the variable of the beambreak
    DigitalChannel beambreak_mid;          // Sets the variable of the beambreak_mid
    Servo leftclaw = null;          // Sets the variable of the Claw
    Servo rightclaw = null;          // Sets the variable of the Claw
    boolean clawstate = false;          // Sets the variable of the clawstate
    boolean toggle = true;          // Sets the variable of the toggle
    public int count = 0;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;
    Arm_Test.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }



    void ForwardorBackwardsCount(double distance, double speed) {

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Driving forward/backwards
        //  double distance= 5; //(in)
        double encodercounts = distance * 60.3686819388;//(1/(75*(1/25.4)))*560;
        int encodercountsint = (int) encodercounts;
        lf.setTargetPosition(encodercountsint);
        lf.setPower(speed);        //Sets the power for the left front wheel
        rf.setTargetPosition(encodercountsint);
        rf.setPower(speed);        //Sets the power for the right front wheel
        lb.setTargetPosition(encodercountsint);
        lb.setPower(speed);        //Sets the power for the left back wheel
        rb.setTargetPosition(encodercountsint);
        rb.setPower(speed);        //Sets the power for the right back wheel
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Rotate(1, 0.5);

        while (lf.isBusy() || rf.isBusy() /*|| lb.isBusy() || rb.isBusy()*/) {
            //run until motors arrive at position
            if(!beambreak.getState()) { //if beam is broken
                conveyor.setPower(1);//Run conveyor
                telemetry.addData("count", count);
                telemetry.addData("toggle", toggle);
                telemetry.addData("beambreak", beambreak.getState());
                telemetry.update();

                if (toggle){ //if toggle is true, or there was no ring in last loop
                    count = count + 1;
                    toggle=false; //set to false to stop count
                    telemetry.addData("count", count);
                    telemetry.addData("toggle", toggle);
                    telemetry.addData("beambreak", beambreak.getState());
                    telemetry.update();
                }
            }
            else{ // if beam break not broken
                toggle=true; //set to false to allow for count next time ring breaks beam
                conveyor.setPower(0); // stop conveyor
                telemetry.addData("count", count);
                telemetry.addData("toggle", toggle);
                telemetry.addData("beambreak", beambreak.getState());
                telemetry.update();
            }

        }
        conveyor.setPower(0); //stops conveyor if loop ended with conveyor running
    }

    void ForwardorBackwards(double distance, double speed) {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Driving forward/backwards
        //  double distance= 5; //(in)
        double encodercounts = distance * 60.3686819388;//(1/(75*(1/25.4)))*560;
        int encodercountsint = (int) encodercounts;
        lf.setTargetPosition(encodercountsint);
        lf.setPower(speed);        //Sets the power for the left front wheel
        rf.setTargetPosition(encodercountsint);
        rf.setPower(speed);        //Sets the power for the right front wheel
        lb.setTargetPosition(encodercountsint);
        lb.setPower(speed);        //Sets the power for the left back wheel
        rb.setTargetPosition(encodercountsint);
        rb.setPower(speed);        //Sets the power for the right back wheel
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Rotate(1, 0.5);

        while (lf.isBusy() || rf.isBusy() /*|| lb.isBusy() || rb.isBusy()*/) {

        }
    }


    void Rotate(double turn, double speed) {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Driving left/right
        //NOT DONE
        double encodercounts = turn * 13.18; // test iteratively
        int encodercountsint = (int) encodercounts;
        lf.setTargetPosition(-encodercountsint);
        lf.setPower(speed);        //
        rf.setTargetPosition(encodercountsint);
        rf.setPower(speed);        //Sets the power for the Long arm
        lb.setTargetPosition(-encodercountsint);
        lb.setPower(speed);        //Sets the power for the Long arm
        rb.setTargetPosition(encodercountsint);
        rb.setPower(speed);        //Sets the power for the Long arm
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (lf.isBusy() || rf.isBusy() /*|| lb.isBusy() || rb.isBusy()*/) {
            //run until motors arrive at position
        }
    }

    void Strafing(double Strafe, double speed) {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Driving left/right
        //Positive is Strafing left negative is Strafing right
        double encodercounts = Strafe * 60.3686819388 * 1.4142135623730950488016887242097;
        int encodercountsint = (int) encodercounts;
        lf.setTargetPosition(-encodercountsint);
        lf.setPower(speed);        //
        rf.setTargetPosition(encodercountsint);
        rf.setPower(speed);        //Sets the power for the Long arm
        lb.setTargetPosition(encodercountsint);
        lb.setPower(speed);        //Sets the power for the Long arm
        rb.setTargetPosition(-encodercountsint);
        rb.setPower(speed);        //Sets the power for the Long arm
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lf.isBusy() || rf.isBusy() /*|| lb.isBusy() || rb.isBusy()*/) {
            //run until motors arrive at position
        }


    }

    boolean IntakeFunction(double speed) {
        intake.setPower(speed);
        if (!beambreak.getState()) {
            conveyor.setPower(speed);
        }
        if (speed == 0) {
            return false;
        } else {
            conveyor.setPower(0);
            return true;
        }

    }

    void ConveyorFunction(double speed) {
        conveyor.setPower(speed);
        intake.setPower(speed);
    }

    void LongArmFunctionDown() {
        double armrotation = MOTOR_TICK_COUNT * (0.4);
        arm.setPower(0.3);        //Sets the power for the Long arm
        arm.setTargetPosition((int) armrotation);        //Tell the motor to go to 90 degrees when told to
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.isBusy()){
            //run until motors arrive at position
        }
    }

    void LongArmFunctionUP() {
        arm.setPower(0.3);        //Sets the power for the Long arm
        arm.setTargetPosition(100);        //Tell the motor to go to 90 degrees when told to
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.isBusy()){
            //run until motors arrive at position
        }
    }

    void ClawClose() {
        leftclaw.setPosition(1);
        rightclaw.setPosition(0);
    }

    void ClawOpen() {
        leftclaw.setPosition(0);
        rightclaw.setPosition(1);
    }

    public void waitForStart() {
    }

    public void runOpMode() {

        displayKind = Arm_Test.DisplayKind.AUTO;

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN;
        blinkinLedDriver.setPattern(pattern);

        lf = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "lf");       //sets the names of the motors on the hardware map
        rf = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "rf");
        lb = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "lb");
        rb = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "rb");
        arm = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "arm");
        shooter = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "shooter");
        intake = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "intake");
        conveyor = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "conveyor");
        beambreak = hardwareMap.get(DigitalChannel.class, "beambreak");
        beambreak_mid = hardwareMap.get(DigitalChannel.class, "beambreak_mid");
        leftclaw = hardwareMap.get(Servo.class, "leftclaw");
        rightclaw = hardwareMap.get(Servo.class, "rightclaw");
        telemetry.addData("count", count);
        telemetry.addData("toggle", toggle);
        telemetry.addData("beambreak", beambreak.getState());
        telemetry.update();

        final double driveP = 2.5;        //PIDF values will change, these are filler values
        final double driveI = 0.1;
        final double driveD = 0.2;
        PIDFCoefficients drivePIDF = new PIDFCoefficients(driveP, driveI, driveD, 0);
        //lf.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
        //rf.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
        //lb.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
        //rb.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setDirection(DcMotorEx.Direction.REVERSE);            //sets the directions of the motors
        lf.setDirection(DcMotorEx.Direction.FORWARD);
        rf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.REVERSE);
        beambreak.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.
        beambreak_mid.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.
        ClawClose();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        //Since this is the first time using the encoder we start it up
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setVelocity(2500);

        ForwardorBackwardsCount(57, 0.75); // Drive forward from wall

        Strafing(-16, -0.5);   // scoot left until aligned with top goal

        ConveyorFunction(1);  //shoot rings in conveyor

        sleep(3500); //time to shoot rings in conveyor

        intake.setPower(1);//begin running intake

        ForwardorBackwardsCount(-14,0.5); //move quickly to ring stack

        ForwardorBackwardsCount(-16, 0.5);    // move slowly to pick up rings, count code in drive while loop





        ConveyorFunction(0); // stop shooting

        telemetry.addData("count", count);
        telemetry.addData("beambreak", beambreak_mid.getState());

        if (count == 0 && beambreak_mid.getState()) {       // if zero rings are picked up, do this portion of code



            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriver.setPattern(pattern);

            ForwardorBackwards(24, 0.75); // move back to line to shoot

            Strafing(12, -0.75);    // Moving to the left the put wobble goal into first box

            LongArmFunctionDown();

            sleep(200);

            ClawOpen();    // Opening Claw

            sleep(150);

            ForwardorBackwards(-50,0.75);   // Going backwards to pick up second wobble goal

            Rotate(90,0.7);

            ForwardorBackwards(7.5, 0.65);    //Going farther the pick up second wobble goal

            ClawClose();    //Closing Claw

            sleep(200);

            Rotate(-88,0.75);  //Rotating to bring second wobble goal into first box

            ForwardorBackwards(43, 1);  //Going really fast to put wobble goal down

            ClawOpen();    //Opening Claw

            sleep(150);

            //LongArmFunctionUP();

            //   Strafing(10, 1);

            //  LongArmFunctionDown();

            ForwardorBackwardsCount(-10, 0.75);
        }
        if (count >= 1 && beambreak_mid.getState()) {

            pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
            blinkinLedDriver.setPattern(pattern);

            ForwardorBackwards(44, 0.75);   // Going to put first wobble goal into second box

            Strafing(-6,0.75);

            LongArmFunctionDown();

            //sleep(240);

            ClawOpen();

            sleep(240);

            //LongArmFunctionUP();//

            ForwardorBackwards(-38,0.75);

            Rotate(183,0.75);

            ForwardorBackwards(6,0.75);

            ConveyorFunction(0.95);

            ClawClose();

            sleep(240);

            shooter.setVelocity(2100);

            Rotate(184, 0.75);

            ForwardorBackwards(40,0.75);

            ClawOpen();

            sleep(100);

            ForwardorBackwardsCount(-4,0.75);
        }
        if (!beambreak_mid.getState()) {

            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriver.setPattern(pattern);

            ForwardorBackwards(24, 0.75); // move back to line to shoot

            ConveyorFunction(1); //begin shooting

            sleep(3500); //time to shoot

            ForwardorBackwards(40, 0.75);   //Going forwards to put Long arm down

            Strafing(7, -1);     // A lining for Long arm

            LongArmFunctionDown();  // Putting wobble goal arm down

            sleep(230);

            ClawOpen();    //Opening the claw after putting down number 1 wobble goal

            sleep(230);

            LongArmFunctionUP();

            ForwardorBackwards(-28,0.75);   //Going back to line
        }

    }
}