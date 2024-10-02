package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slides {
    //establish left and right slide for gamepad
    public final DcMotor leftSlide, rightSlide;
    public final Gamepad gamepad2;
    //This determines where slides will stop depending on what the driver wants
    public static int HIGH = 450;
    public static int MID =358;
    public static int LOW =2;
    public static int INTAKE = 0;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Slides(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad2 = opMode.gamepad2;
        this.telemetry = opMode.telemetry;
        leftSlide = (DcMotor) hardwareMap.get("leftSlide");
        rightSlide = (DcMotor) hardwareMap.get("rightSlide");

        //The slides must be set to correct directions
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //brake
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //setting power, power can vary 0-1
        leftSlide.setPower(1);
        rightSlide.setPower(1);
        //
        rightSlide.setTargetPosition(INTAKE);
        leftSlide.setTargetPosition(INTAKE);
    }
    public void teleOp() {
        if (gamepad2.dpad_up) slidesHighBasket();
        else if (gamepad2.dpad_right) slidesLowBasket();
        else if (gamepad2.dpad_left) slidesLowChamber();
        else reset();
        slideManual();

        if (rightSlide.getCurrentPosition() ==0 && leftSlide.getCurrentPosition() == 0) {
            gamepad2.rumble(10);
        }

        telemetry.addData("The right slide position in TICKS is: ", rightSlide.getCurrentPosition());
        telemetry.addData("The left slide position in TICKS is: ", leftSlide.getCurrentPosition());

    }
    public void slidesHighBasket() {
        rightSlide.setTargetPosition(HIGH);
        leftSlide.setTargetPosition(HIGH);
        telemetry.update();

    }
    public void slidesLowBasket() {
        //Use this for high chamber
        rightSlide.setTargetPosition(MID);
        leftSlide.setTargetPosition(MID);
        telemetry.update();

    }
    public void slidesLowChamber() {
        rightSlide.setTargetPosition(LOW);
        leftSlide.setTargetPosition(LOW);
        telemetry.update();

    }
    public void reset() {
        rightSlide.setTargetPosition(INTAKE);
        leftSlide.setTargetPosition(INTAKE);
        telemetry.update();

    }
    public void slideManual() {
        int multiplier = (int) -gamepad2.left_stick_y;

        if (multiplier != 0) {
            for (int i = 50; i < 1000; i++) {
                leftSlide.setTargetPosition(multiplier * i++);
                rightSlide.setTargetPosition(multiplier * i++);
            }
        }
    /*
    -1 >= gamepad2.left_stick_y
                && gamepad2.left_stick_y <=1
                && gamepad2.left_stick_y !=0
     */
    }
}
