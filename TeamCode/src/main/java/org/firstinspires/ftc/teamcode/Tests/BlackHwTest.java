package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Components.Carousel;
import org.firstinspires.ftc.teamcode.Components.LedColor;
@Disabled

@Autonomous(name = "Black Hardware Test")

public class BlackHwTest extends LinearOpMode {

    DcMotor  [] dcMotor = null;


    static final int NOOFMOTORS     = 7;       // number of motor in robot

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
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

        dcMotor = new DcMotor[NOOFMOTORS];


        // Define and Initialize Motors
        dcMotor[0] = hwMap.get(DcMotor.class, "motorLeftFront");
        dcMotor[1] = hwMap.get(DcMotor.class, "motorRightFront");
        dcMotor[2] = hwMap.get(DcMotor.class, "motorLeftBack");
        dcMotor[3] = hwMap.get(DcMotor.class, "motorRightBack");
        dcMotor[4] = hwMap.get(DcMotor.class, "IntakeMotor");
        dcMotor[5] = hwMap.get(DcMotor.class, "ShooterMotor");
        dcMotor[6] = hwMap.get(DcMotor.class, "TransferMotor");


    }

    public void forward(DcMotor  motorDrive, double power, long time) {
        motorDrive.setPower(power);
        // Set all motors to zero power
        sleep(time);
        motorDrive.setPower(0);
    }

    public void backward(DcMotor  motorDrive, double power, long time) {
        motorDrive.setPower(-power);
        // Set all motors to zero power
        sleep(time);
        motorDrive.setPower(0);
    }
    @Override
    public void runOpMode() {
        long sleepTime = 1000;
        myinit(hardwareMap);
        led = new LedColor(this);
        for (int i=1; i<=4;i++) {
            telemetry.addData("Led" + i," Changing Color");
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
        waitForStart();
        for (int i = 0; i < NOOFMOTORS; i++) {
            String caption;
            caption = "Motor" + i;
            telemetry.addData(caption," Moving Forward");
            telemetry.update();
            forward(dcMotor[i],0.5,sleepTime);

            telemetry.addData(caption," Moving Backward");
            telemetry.update();
            backward(dcMotor[i],0.5,sleepTime);

        }


/*WORKING BLINKIN CODE
 * Configure the driver on a servo port, and name it "blinkin".
        RevBlinkinLedDriver blinkinLedDriver;
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        * use one of the patttern
        *
        *
        // CP1: Color 1 Pattern

        CP1_END_TO_END_BLEND_TO_BLACK,
        CP1_LARSON_SCANNER,
        CP1_LIGHT_CHASE,
        CP1_HEARTBEAT_SLOW,
        CP1_HEARTBEAT_MEDIUM,
        CP1_HEARTBEAT_FAST,
        CP1_BREATH_SLOW,
        CP1_BREATH_FAST,
        CP1_SHOT,
        CP1_STROBE,

         //CP2: Color 2 Pattern

        CP2_END_TO_END_BLEND_TO_BLACK,
        CP2_LARSON_SCANNER,
        CP2_LIGHT_CHASE,
        CP2_HEARTBEAT_SLOW,
        CP2_HEARTBEAT_MEDIUM,
        CP2_HEARTBEAT_FAST,
        CP2_BREATH_SLOW,
        CP2_BREATH_FAST,
        CP2_SHOT,
        CP2_STROBE,

        //CP1_2: Color 1 and 2 Pattern

        CP1_2_SPARKLE_1_ON_2,
        CP1_2_SPARKLE_2_ON_1,
        CP1_2_COLOR_GRADIENT,
        CP1_2_BEATS_PER_MINUTE,
        CP1_2_END_TO_END_BLEND_1_TO_2,
        CP1_2_END_TO_END_BLEND,
        CP1_2_NO_BLENDING,
        CP1_2_TWINKLES,
        CP1_2_COLOR_WAVES,
        CP1_2_SINELON,

        // Solid color

        HOT_PINK,
        DARK_RED,
        RED,
        RED_ORANGE,
        ORANGE,
        GOLD,
        YELLOW,
        LAWN_GREEN,
        LIME,
        DARK_GREEN,
        GREEN,
        BLUE_GREEN,
        AQUA,
        SKY_BLUE,
        DARK_BLUE,
        BLUE,
        BLUE_VIOLET,
        VIOLET,
        WHITE,
        GRAY,
        DARK_GRAY,
        BLACK;

        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);



 */
        Carousel carousel = new Carousel(this);
        carousel.spinCarouselAutonomousBlue();
        carousel.spinCarouselAutonomousRed();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            carousel.spinCarouselAutonomousBlue();
            carousel.spinCarouselAutonomousRed();

 /*           // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                    break;
                }
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.update();

            // Set the servo to the new position and pause;
            carousel.setPosition(position);
            sleep(CYCLE_MS);
            idle();
 */
        }


        stop();
    }


}
