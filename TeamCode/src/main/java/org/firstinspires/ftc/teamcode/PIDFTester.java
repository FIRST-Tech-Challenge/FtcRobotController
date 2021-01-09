package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Created by STEM Punk on 2/23/2020.
 */

//@TeleOp(name="Omni: PIDFTester", group ="TeleOp")
public class PIDFTester extends OpMode {

    private double driverAngle = 0.0;
    private final double MAX_SPEED = 1.0;
    private final double MAX_SPIN = 1.0;
    private double speedMultiplier = MAX_SPEED;
    private double spinMultiplier = MAX_SPIN;
    private boolean aHeld = false;
    private boolean bHeld = false;
    private boolean yHeld = false;
    private boolean xHeld = false;
    private boolean upHeld = false;
    private boolean downHeld = false;
    private boolean rightHeld = false;
    private boolean leftHeld = false;
    private boolean rightBumperHeld = false;
    private boolean leftBumperHeld = false;
    private boolean a2Held = false;
    private boolean b2Held = false;
    private boolean y2Held = false;
    private boolean x2Held = false;
    private boolean up2Held = false;
    private boolean down2Held = false;
    private boolean aPressed;
    private boolean bPressed;
    private boolean yPressed;
    private boolean xPressed;
    private boolean upPressed;
    private boolean downPressed;
    private boolean rightPressed;
    private boolean leftPressed;
    private boolean rightBumperPressed;
    private boolean leftBumperPressed;
    private boolean a2Pressed;
    private boolean b2Pressed;
    private boolean y2Pressed;
    private boolean x2Pressed;
    private boolean up2Pressed;
    private boolean down2Pressed;
    private double yPower;
    private double xPower;
    private double spin;
    private boolean lifting = false;
    private boolean clawPinched = false;
    private ElapsedTime loopTime = new ElapsedTime();

    public static double CLAW_OPEN = 0.30;
    public static double CLAW_PINCHED = 0.95;
    // Hardware devices
    public DcMotorEx lifter = null;
    protected Servo claw = null;
    List<LynxModule> allHubs;

    // Hardware config names.
    public final static String HUB1 = "Expansion Hub 2";
    public final static String HUB2 = "Expansion Hub 3";
    public final static String LIFTER = "Lifter";
    public final static String CLAW = "Claw";

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        claw = hardwareMap.get(Servo.class, CLAW);
        lifter = hardwareMap.get(DcMotorEx.class, LIFTER);
        lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw.setPosition(CLAW_OPEN);
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        yPower = -HardwareOmnibot.cleanMotionValues(gamepad1.left_stick_y);
        xPower = HardwareOmnibot.cleanMotionValues(gamepad1.left_stick_x);
        spin = -HardwareOmnibot.cleanMotionValues(gamepad1.right_stick_x);
        aPressed = gamepad1.a;
        bPressed = gamepad1.b;
        yPressed = gamepad1.y;
        xPressed = gamepad1.x;
        upPressed = gamepad1.dpad_up;
        downPressed = gamepad1.dpad_down;
        rightPressed = gamepad1.dpad_right;
        leftPressed = gamepad1.dpad_left;
        rightBumperPressed = gamepad1.right_bumper;
        leftBumperPressed = gamepad1.left_bumper;
        a2Pressed = gamepad2.a;
        b2Pressed = gamepad2.b;
        y2Pressed = gamepad2.y;
        x2Pressed = gamepad2.x;
        up2Pressed = gamepad2.dpad_up;
        down2Pressed = gamepad2.dpad_down;

        if (!xHeld && xPressed) {
            xHeld = true;
        } else if (!xPressed) {
            xHeld = false;
        }

        if (!rightBumperHeld && rightBumperPressed) {
            rightBumperHeld = true;
        } else if (!rightBumperPressed) {
            rightBumperHeld = false;
        }

        if (!leftBumperHeld && leftBumperPressed) {
            leftBumperHeld = true;
        } else if (!leftBumperPressed) {
            leftBumperHeld = false;
        }

        if (!aHeld && aPressed) {
            if(!lifting) {
                lifting = true;
                lifter.setPower(1.0);
            } else {
                lifter.setPower(0.0);
                lifting = false;
            }
            aHeld = true;
        } else if (!aPressed) {
            aHeld = false;
        }

        if (!bHeld && bPressed) {
            if(!clawPinched) {
                clawPinched = true;
                claw.setPosition(CLAW_PINCHED);
            } else {
                claw.setPosition(CLAW_OPEN);
                clawPinched = false;
            }
            bHeld = true;
        } else if (!bPressed) {
            bHeld = false;
        }

        if (!yHeld && yPressed) {
            yHeld = true;
        } else if (!yPressed) {
            yHeld = false;
        }

        if (!upHeld && upPressed) {
            upHeld = true;
        } else if (!upPressed) {
            upHeld = false;
        }

        if (!downHeld && downPressed) {
            downHeld = true;
        } else if (!downPressed) {
            downHeld = false;
        }

        if (!rightHeld && rightPressed) {
            rightHeld = true;
        } else if (!rightPressed) {
            rightHeld = false;
        }

        if (!leftHeld && leftPressed) {
            leftHeld = true;
        } else if (!leftPressed) {
            leftHeld = false;
        }
        if(!a2Held && a2Pressed)
        {
            a2Held = true;
        } else if(!a2Pressed) {
            a2Held = false;
        }

        if(!b2Held && b2Pressed)
        {
            b2Held = true;
        } else if(!b2Pressed) {
            b2Held = false;
        }

        if(!y2Held && y2Pressed)
        {
            y2Held = true;
        } else if(!y2Pressed) {
            y2Held = false;
        }

        if(!x2Held && x2Pressed)
        {
            x2Held = true;
        } else if(!x2Pressed) {
            x2Held = false;
        }

        if(!up2Held && up2Pressed)
        {
            up2Held = true;
        } else if (!up2Pressed) {
			up2Held = false;
		}

        if(!down2Held && down2Pressed)
        {
            down2Held = true;
        } else if (!down2Pressed) {
			down2Held = false;
		}

        PIDFCoefficients myLiftMotor = lifter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients myLiftMotor2 = lifter.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Motor RTP P: ", myLiftMotor2.p);
        telemetry.addData("Motor RUE P: ", myLiftMotor.p);
        telemetry.addData("Motor RUE I: ", myLiftMotor.i);
        telemetry.addData("Motor RUE D: ", myLiftMotor.d);
        telemetry.addData("Motor RUE F: ", myLiftMotor.f);
        telemetry.addData("Allowed Error: ", lifter.getTargetPositionTolerance());
        telemetry.addData("Encoder Position: ", lifter.getCurrentPosition());
        updateTelemetry(telemetry);
    }
}
