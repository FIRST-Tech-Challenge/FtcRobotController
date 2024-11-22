package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.WrapRads;
import static org.firstinspires.ftc.teamcode.Utilize.toRadian;
import static org.firstinspires.ftc.teamcode.Utilize.toDegree;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Tele")
public  class Tele extends Robot {

    private Controller controller;

    // Variables
    int targetLift = 0;
    double setpoint = 0, H_Ang = 0, AL_Ang = 0, AD_Ang = 0;
    boolean autoLift = false, V_Pressed = false, VisBusy = false, ITisOn = false, tp_Pressed = false,
            H_disable = false, r_disable = false, hl_Pressed = false, ll_Pressed = false, ADC_Pressed = false,
            ADCisON = false, B_Pressed = false, BisON = false, Right_isTouch = false,Auto_Lift = false;
    double CurrentTime = System.nanoTime() * 1E-9,  lastRXtime = CurrentTime;

    private void Init() {
        // Initialize Robot
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{0, 0, AL_Ang},
                new double[]{0, 0, 0, 0});

        controller = new Controller(1.0, 0.05, 0.1, 0 , 0.2, toRadian(0.75));


        setpoint = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    }

    private void Movement() {
        CurrentTime = System.nanoTime() * 1E-9;
        double speed = 0.275;
        double lx = -gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double x1 = gamepad1.dpad_left ? speed : gamepad1.dpad_right ? speed : lx;
        double y1 = gamepad1.dpad_up ? speed : gamepad1.dpad_down ? -speed : ly;
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double x2 = (Math.cos(yaw) * x1) - (Math.sin(yaw) * y1);
        double y2 = (Math.sin(yaw) * x1) + (Math.cos(yaw) * y1);
        // Rotate
        double r = r_disable ? 0 : controller.Calculate(WrapRads(setpoint - yaw));
        double x = -gamepad1.right_stick_x;
        if (x != 0 || CurrentTime - lastRXtime < 0.45) {
            r = x;
            setpoint = yaw;
        }
        if (lx == 0 && ly == 0 && x== 0 && Math.abs(r) < 0.2)  r = 0;
        lastRXtime = x != 0 ? CurrentTime : lastRXtime;
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 0.5);
        MovePower((y2 - x2 - r) / d, (y2 + x2 + r) / d,
                (y2 + x2 - r) / d, (y2 - x2 + r) / d);
        telemetry.addData("yaw", toDegree(yaw));
        telemetry.addData("setpoint", toDegree(setpoint));
        telemetry.addData("error", controller.Error);
    }

    private void Lift() {
        double LT = gamepad1.right_trigger;
        double RT = gamepad1.left_trigger;
        boolean LT_Press = LT > 0.25;
        boolean RT_Press = RT > 0.25;
        double CurPos = Math.max(LL.getCurrentPosition(), RL.getCurrentPosition());
        boolean du = gamepad2.dpad_up;
        boolean dl = gamepad2.dpad_left;
        boolean dd = gamepad2.dpad_down;
        Right_isTouch = RTS.isPressed();

        if (Right_isTouch) {
            LL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

//        double Pos = Auto_Arm && LT < 0.5 ? arm : (Math.max(LL.getCurrentPosition(), RL.getCurrentPosition()) > LiftPos - 500 ? 1 : 0);
        double sp = LT > 0.25 ? LT : RT > 0.25 ? -RT : 0;

        double power = (Right_isTouch  && RT > 0.25 ? 0 : CurPos > 2800 && LT > 0.25 ? 0 : sp);

        LiftPower(power);

//        if (spL == 0 && spR == 0) {
//            LL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }

        telemetry.addData("Lift", CurPos);


    }
    private void Drop() {
        boolean dp = gamepad1.b;
        if (!(dp)) {
            B_Pressed = false;
            return;
        }
        if (B_Pressed) return;
        B_Pressed = true;
        if (!BisON) {
            SetServoPos(0, Claw);
            sleep(50);
            SetServoPos(0.3, LA, RA);
            SetServoPos(0, LJ, RJ);
            BisON = true;
            return;
        }
        SetServoPos(1, LJ, RJ);
        BisON = false;
    }

    private void AdjustClaw() {
        boolean adc = gamepad1.y;
        if (!(adc)) {
            ADC_Pressed = false;
            return;
        }
        if (ADC_Pressed) return;
        ADC_Pressed = true;
        if (!ADCisON) {
            SetServoPos(0.3, RC);
            ADCisON = true;
            return;
        }
        SetServoPos(0.1, RC);
        ADCisON = false;
    }

    private void Lowerclaw() {
        boolean lc = gamepad1.right_bumper;
        if (!(lc)) {
            V_Pressed = false;
            return;
        }
        if (V_Pressed) return;
        V_Pressed = true;
        if (!VisBusy) {
            SetServoPos(1, LA, RA);
            VisBusy = true;
            return;
        }
        SetServoPos(0.9, LA, RA);
        VisBusy = false;
    }

    private void Frontarm() {
        boolean tp = gamepad1.a;
        if (!(tp)) {
            tp_Pressed = false;
            return;
        }
        if (tp_Pressed) return;
        tp_Pressed = true;
        if (!ITisOn) {
            SetServoPos(0, Claw);
            SetServoPos(0.5, Ll, Rl);
            SetServoPos(0.9, LA, RA);
            SetServoPos(0.5, ADL, ADR);
            ITisOn = true;
            return;
        }
        SetServoPos(0.25, Claw);
        sleep(200);
        SetServoPos(0.15, Claw);
        SetServoPos(0, Ll, Rl);
        SetServoPos(0.25, LA, RA);
        SetServoPos(0.05, RC);
        SetServoPos(1, ADL, ADR);
        SetServoPos(1, LJ, RJ);
        ITisOn = false;
    }
    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Odomentry();
                Movement();
                Lift();
                Frontarm();
                Lowerclaw();
                AdjustClaw();
                Drop();

//                telemetry.addData("XYH", "%6f cm %6f cm", Posx, Posy);
                telemetry.addData("LRM", "%6d  %6d %6d", left_encoder_pos, right_encoder_pos, center_encoder_pos);
                telemetry.addData("heading", toDegree(heading));
                telemetry.addData("XYH", "%6f cm %6f cm", Posx, Posy);
                telemetry.update();
                if(gamepad1.back) {
                    imu.resetYaw();
                    setpoint = 0;
                }
                }

            }
        }
    }
