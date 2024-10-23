package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Photon
public class Slides {

    //establish left and right slide for gamepad
    public final DcMotor leftSlide, rightSlide;
    public final Gamepad gamepad2;
    //This determines where slides will stop depending on what the driver wants
    private static int HIGH = 1000;
    public static int MID = 500;
    public static int LOW = 0;
    public static int INTAKE = 0;
    private static double WHEEL_DIAMETER = 1.77;//in
    private static double TICKS_PER_REV = 384.539792388;
    private static double GEAR_RATIO = 1;
    private static double ticks;

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
    public void teleOp(Gamepad controller) {
        if (controller.dpad_up) slidesHighBasket();
        else if (controller.dpad_right) slidesLowBasket();
        else if (controller.dpad_left) slidesLowChamber();
        else if (controller.dpad_down) reset();

        telemetry.addData("The right slide position in TICKS is: ", rightSlide.getCurrentPosition());
        telemetry.addData("The left slide position in TICKS is: ", leftSlide.getCurrentPosition());

    }
    public void slidesHighBasket() {
        rightSlide.setTargetPosition(HIGH);
        leftSlide.setTargetPosition(HIGH);
    }
    public void slidesLowBasket() {
        //Use this for high chamber
        rightSlide.setTargetPosition(MID);
        leftSlide.setTargetPosition(MID);
    }
    public void slidesLowChamber() {
        rightSlide.setTargetPosition(LOW);
        leftSlide.setTargetPosition(LOW);
    }
    public void reset() {
        rightSlide.setTargetPosition(INTAKE);
        leftSlide.setTargetPosition(INTAKE);

    }
    public void slideManual(Gamepad gamepad) {
        double multiplier = -gamepad.left_stick_y;

        if (multiplier >= 0.25) {
            for (int i = 50; i < 1000; i++) {
                leftSlide.setTargetPosition((int) (leftSlide.getCurrentPosition() - (multiplier * i++)));
                rightSlide.setTargetPosition((int) (rightSlide.getCurrentPosition() - (multiplier * i++)));
            }
        } else if (multiplier <= -0.25) {
            for (int i = -50; i < 0; i--) {
                leftSlide.setTargetPosition((int) (leftSlide.getCurrentPosition() - (multiplier * i--)));
                rightSlide.setTargetPosition((int) (rightSlide.getCurrentPosition() - (multiplier * i--)));
            }
        }
    }
}
