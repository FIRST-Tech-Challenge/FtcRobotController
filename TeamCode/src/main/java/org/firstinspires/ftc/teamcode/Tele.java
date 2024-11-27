package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.WrapRads;
import static org.firstinspires.ftc.teamcode.Utilize.toRadian;
import static org.firstinspires.ftc.teamcode.Utilize.toDegree;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Tele")
public  class Tele extends Robot {

    private Controller controller;

    // Variables
    int targetLift = 0;
    double setpoint = 0, H_Ang = 0, AL_Ang = 0, LiftPos = 0;
    boolean  V_Pressed = false, VisBusy = false, ITisOn = false, tp_Pressed = false,
             r_disable = false, R_Pressed = false,RisON = false, ADC_Pressed = false,
             ADCisON = false, B_Pressed = false, BisON = false, Right_isTouch = false,
             Auto_Lift = false ,a_press = false, On_Lift = false, L_press = false,
             LisON = false;
    ;
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
        double lift = Math.max(LL.getCurrentPosition(), RL.getCurrentPosition());
        double speed = 0.4;
        double lx = -gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double x1 = gamepad1.dpad_right ? -speed : gamepad1.dpad_left ? speed : lx;
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
        double l = lift > 2000 ? 0.8 : 1;
        double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 0.5);
        MovePower(((y2 - x2 - r) / d) * l, ((y2 + x2 + r) / d) * l,
                  ((y2 + x2 - r) / d) * l,  ((y2 - x2 + r) / d) * l);
        telemetry.addData("yaw", toDegree(yaw));
        telemetry.addData("setpoint", toDegree(setpoint));
        telemetry.addData("error", controller.Error);
    }

    private void Lift() {
        double RT = gamepad1.right_trigger;
        double LT = gamepad1.left_trigger;
        boolean LT_Press = LT > 0.25;
        boolean RT_Press = RT > 0.25;
        boolean lb = gamepad1.left_bumper;
        boolean ls = gamepad1.left_stick_button;
        double CurPos = Math.max(LL.getCurrentPosition(), RL.getCurrentPosition());
        Right_isTouch = RS.isPressed();

        if (Right_isTouch && !On_Lift) {
            LL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (RT > 0 || Auto_Lift) {
            LL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (RT_Press ) {
            SetServoPos(0.68, LJ, RJ);
            SetServoPos(0, Claw);
        }

        if (LT_Press) {
            SetServoPos(0.2, LA, RA);
            SetServoPos(0.9, LJ, RJ);

            BisON = false;
        }

        if (gamepad1.right_stick_button) {

            Auto_Lift = true;
            LiftPos = High_Chamber;

        }



//        double Pos = Auto_Arm && LT < 0.5 ? arm : (Math.max(LL.getCurrentPosition(), RL.getCurrentPosition()) > LiftPos - 500 ? 1 : 0);
        double sp = RT > 0.25 ? RT : LT > 0.25 ? -LT : 0;

        double power =  On_Lift ? -0.2 : Auto_Lift ? ((CurPos < (LiftPos + 50) && CurPos > (LiftPos - 50)) ? 0 : CurPos > LiftPos ? -0.8 : 1) :
                        (Right_isTouch  && LT > 0.25 ? 0 : CurPos > 2900 && RT > 0.25 ? 0 : sp );

        LiftPower(power);

        if (power == 0) {
            LL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (Lift_Power ==0) {
            Auto_Lift = false;
        }


        telemetry.addData("Lift", CurPos);

        if (!(lb)) {
            a_press = false;
            return;
        }
        if (a_press) return;
        a_press = true;
        if (!(On_Lift)) {
            On_Lift = true;
            return;
        }
        On_Lift = false;



    }

//    private void Claw() {
//        boolean cl = gamepad1.right_stick_button;
//        double LT = gamepad1.left_trigger;
//        boolean LT_Press = LT > 0.25;
//        if (!(cl)) {
//            R_Pressed = false;
//            return;
//        }
//        if (R_Pressed) return;
//        R_Pressed = true;
//        if (!RisON && !ITisOn || LT_Press) {
//            SetServoPos(0.08, LA, RA);
//            SetServoPos(0.68, LJ, RJ);
//            SetServoPos(0, Claw);
//
//            RisON = true;
//            return;
//        }
//        SetServoPos(0, LA, RA);
//        SetServoPos(1, LJ, RJ);
//        RisON = false;
//    }

    private void Drop() {
        boolean dp = gamepad1.b;
        if (!(dp)) {
            B_Pressed = false;
            return;
        }
        if (B_Pressed) return;
        B_Pressed = true;
        if (!BisON ) {
            SetServoPos(0.2, LA, RA);
            SetServoPos(0, LJ, RJ);
            BisON = true;
            return;
        }
        SetServoPos(0.9, LJ, RJ);
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
            SetServoPos(0.65, RC);
            ADCisON = true;
            return;
        }
        SetServoPos(0.12, RC);
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
            SetServoPos(0.75, LA, RA);
            VisBusy = true;
            return;
        }
        SetServoPos(0.67, LA, RA);
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
            SetServoPos(0.9, LJ, RJ);
            SetServoPos(0, Claw);
            SetServoPos(0.6, Ll, Rl);
            SetServoPos(0.67, LA, RA);
            SetServoPos(0.55, ADL, ADR);

            AdjustClaw();
            Lowerclaw();
            ITisOn = true;
            return;
        }
        SetServoPos(0.4, Claw);
        sleep(180);
        SetServoPos(0.35, Claw);
        SetServoPos(0, Ll, Rl);
        SetServoPos(0, LA, RA);
        SetServoPos(0.12, RC);
        SetServoPos(0, ADL, ADR);
        SetServoPos(0.9, LJ, RJ);
        ITisOn = false;
    }
    private void FrontarmP() {
        boolean kl = gamepad1.x;
        if (!(kl)) {
            R_Pressed = false;
            return;
        }
        if (R_Pressed) return;
        R_Pressed = true;
        if (!RisON ) {
            SetServoPos(0.6, Ll, Rl);
            SetServoPos(0.75, LA, RA);
            SetServoPos(0.55, ADL, ADR);

            RisON = true;
            return;
        }
        SetServoPos(0, Claw);
        SetServoPos(0, Ll, Rl);
        SetServoPos(0.35, LA, RA);
        SetServoPos(0.12, RC);
        SetServoPos(0, ADL, ADR);

        RisON = false;
    }

    @Override
    public void runOpMode() {
        Init();

        while (!(RS.isPressed())) {
            double power = RS.isPressed() ? 0 : -0.4;
            LiftPower(power);
        }
        LiftPower(0);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Odomentry();
                Movement();
                Lift();
                Frontarm();
                Drop();
                Lowerclaw();
                AdjustClaw();
                FrontarmP();

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
