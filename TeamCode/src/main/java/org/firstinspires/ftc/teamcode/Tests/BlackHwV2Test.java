package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.LedColor;

@Autonomous(name = "Black Hardware V2 Test")

public class BlackHwV2Test extends LinearOpMode {

    CRServo  [] crServo1 = null;
    DcMotor  [] motorRight = null;
    DcMotor  [] motorLeft = null;
    CRServo     crServo;
    DcMotor   intakeMotor;
    DcMotor   turretExtension;
    DcMotor   turretRotation;
   // DcMotor []  dcMotor;
    Servo     servo;



    static final int NOOFMOTORS       = 2;       // number of motor in robot
    static final int NOOFCRSERVOS     = 1;       // number of CR servos in robot
    static final int NOOFSERVOS       = 1;       // number of servos in robot

    static final double ROTATIONS_PER_INCH   = 0.5;     // amount to slew servo each CYCLE_MS cycle
    static final double INCREMENT   = 0.5;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    // Define class members
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    LedColor led;

    /* Initialize standard Hardware interfaces */
    public void myinit(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        motorRight = new DcMotorEx[NOOFMOTORS];
        motorLeft = new DcMotorEx[NOOFMOTORS];

        // Define and Initialize Motors
        motorRight[0] = hwMap.get(DcMotor.class, "motorRightFront");
        motorRight[0].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorRight[1] = hwMap.get(DcMotor.class, "motorRightBack");
        motorRight[1].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorLeft[0] = hwMap.get(DcMotor.class, "motorLeftFront");
        motorLeft[0].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorLeft[1] = hwMap.get(DcMotor.class, "motorLeftBack");
        motorLeft[1].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intakeMotor = hwMap.get(DcMotor.class, "IntakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        turretExtension = hwMap.get(DcMotor.class, "turret_Extension");
        turretExtension.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        turretRotation = hwMap.get(DcMotor.class, "turret_Rotation");
        turretRotation.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //crServo = new CRServo[NOOFCRSERVOS];
        //crServo[0] = hwMap.get(CRServo.class, "carousel");

        crServo = hwMap.get(CRServo.class, "carousel"); //
        //servo = hwMap.get(Servo.class, "tsedepositer");


    }

    public void forwardFrontDc( double power, long time) {
        for (int i =0; i< NOOFMOTORS; i++) {
            motorRight[i].setPower(power);
            motorLeft[i].setPower(power);
        }
        sleep(time);
        for (int i =0; i< NOOFMOTORS; i++) {
            motorRight[i].setPower(0);
            motorLeft[i].setPower(0);
            // Set all motors to zero power
        }
    }

    public void testMotor(String name, DcMotor  motorDrive, double power, long time) {
        String caption;

        caption = "Motor" + name;
        telemetry.addData(caption," Moving Forward");
        telemetry.update();
        motorDrive.setPower(power);
        sleep(time);
        motorDrive.setPower(0);
        sleep(time);
        telemetry.addData(caption," Moving Backward");
        telemetry.update();
        motorDrive.setPower(-power);
        sleep(time);
        motorDrive.setPower(0);

    }

    public void testLed(int sleepTime) {
        led = new LedColor(this);
        for (int i = 1; i <= 4; i++) {
            telemetry.addData("Led" + i, " Changing Color");
            telemetry.update();
            led.LedRed(i);
            sleep(sleepTime);
            led.LedGreen(i);
            sleep(sleepTime);
            led.LedAmber(i);
            sleep(sleepTime);
            led.LedOff(i);
            sleep(sleepTime);

        }
    }

    public void testCRServo(String  name, CRServo crServo, double power, long time) {
        String caption;
        caption = "Servo" + name;
        telemetry.addData(caption," Moving Forward");
        telemetry.update();
        crServo.setPower(power);
        idle();
        sleep(time);
        crServo.setPower(0);
        idle();
        sleep(time);
        telemetry.addData(caption," Moving backward");
        telemetry.update();
        crServo.setPower(-power);
        idle();
        sleep(time);
        crServo.setPower(0);
    }

    public void testTseDepositerTape(String name, Servo servo, double inch, double rotation_per_inch, double reverse) {
        String caption;

        caption = "Servo" + name + reverse;
        telemetry.addData(caption," Moving ");
        telemetry.update();

        double position = inch*rotation_per_inch;

        for (int i =0;  (position> 0.0); i++) {
            double amountRotated = 0.0;
                servo.getController().setServoPosition(servo.getPortNumber(),0.0 + reverse);
                //every inch
                if (position < MAX_POS) {
                    if (reverse == 1.0) {
                        servo.setPosition(MAX_POS - position);
                    } else {
                        servo.setPosition(position);
                    }
                    amountRotated += position;
                    position = 0.0;
                    sleep(250);
                } else {
                    if (reverse == 1.0) {
                        servo.setPosition(MIN_POS);
                    } else {
                        servo.setPosition(MAX_POS);
                    }

                    position = position - MAX_POS;
                    amountRotated += MAX_POS;
                    sleep(250);
                }
                telemetry.addData("Servo Position", "%5.2f", amountRotated);
                telemetry.update();

        }
    }

    @Override
    public void runOpMode() {
        long sleepTime = 1000;
        myinit(hardwareMap);
        waitForStart();
    /**/    telemetry.addData("motor","foward");
        telemetry.update();
        forwardFrontDc( 0.10,1000);
        sleep(1000);
        telemetry.addData("motor","backward");
        telemetry.update();
        forwardFrontDc( -0.10,1000);
        testMotor("turretExtension",turretExtension,0.10,1000);

        testMotor("turretRotation",turretRotation,0.10,1000);
        testMotor("intakeMotor",intakeMotor,0.10,1000);
        testCRServo("carousel", crServo,-0.10,1000);
//*/

/*        testTseDepositerTape("forward",servo,10,1.3, 0.0);
        testTseDepositerTape("backward",servo,10,1.3, 1.0); */
/*        testLed(500);

        for (int i = 0; i < NOOFMOTORS; i++) {
            testMotor(Integer.toString(i), dcMotor[i], 0.5, sleepTime);
        }

        for (int i = 0; i < NOOFSERVOS; i++) {
            testCRServo(Integer.toString(i), crServo[i], 0.5, sleepTime);
        }

  */     //*  */

       stop();
    }


}
