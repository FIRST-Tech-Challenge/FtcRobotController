package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class Robot extends LinearOpMode {
    public IMU imu;
    public VisionPortal visionPortal;
    public Servo Ll, Rl, LA, RA, RC, Claw, LJ, RJ, ADL, ADR;
    public DcMotorEx FL, FR, BL, BR, RL, LL,  encoder1, encoder2, encoder3 ;
    public TouchSensor RS;
    public int FL_Target, FR_Target, BL_Target, BR_Target;
    public final double[] tileSize            = {60.96, 60.96};  // Width * Length
    /* TETRIX Motor Encoder per revolution */
    public final int      Counts_per_TETRIX   = 24;
    /** HD HEX Motor Encoder per revolution */
    public final int      Counts_per_HD_HEX   = 28;
    /** 20:1 HD HEX Motor Encoder per revolution */
    public final int      Gear_20_HD_HEX      = Counts_per_HD_HEX * 20;
    /** (3 * 4 * 5):1 UltraPlanetary HD HEX Motor Encoder per revolution */
    public final double   Gear_60_HD_HEX      = Counts_per_HD_HEX * 54.8;
    public final double   Wheel_Diameter_Inch = 7.5/2.54;
    public final double   Counts_per_Inch     = Gear_20_HD_HEX / (Wheel_Diameter_Inch * Math.PI);
    public double[]       currentXY           = {0, 0};
    public final double   L                   = 33.4; //distance between 1 and 2 in cm
    public final double   B                   = 9.65; //distance between center of 1 and 2 and 3 in cm
    public final double   r                   = 2.4 ; // Odomentry wheel radius in cm
    public final double   N                   = 2000.0 ; // ticks per one rotation
    public double         cm_per_tick     = 2.0 * Math.PI * r / N ;
    public double         theta, Posx, Posy, heading, n, CurPosLift, Lift_Power, dn1, dn2, dn3, dyaw ;
    //    // update encoder
    int                   left_encoder_pos , right_encoder_pos , center_encoder_pos ,
            prev_left_encoder_pos, prev_right_encoder_pos, prev_center_encoder_pos = 0;
    double                CurrentYaw, OldYaw         = 0;
    private double Current_Time = System.nanoTime() * 1E-9;
    private double Last_Time = Current_Time;
    private double Last_yaw;

    public final int Low_Chamber  = 1000;
    public final int High_Chamber = 850;
    public final int High_Basket  = 3000;



    public void Odomentry() {
        left_encoder_pos = -encoder1.getCurrentPosition();
        right_encoder_pos = -encoder2.getCurrentPosition();
        center_encoder_pos = encoder3.getCurrentPosition();

        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        heading = yaw;

        double delta_left_encoder_pos = (left_encoder_pos - prev_left_encoder_pos) * cm_per_tick;
        double delta_right_encoder_pos = (right_encoder_pos - prev_right_encoder_pos) * cm_per_tick;
        double delta_center_encoder_pos = (center_encoder_pos - prev_center_encoder_pos) * cm_per_tick;

//        double phi = (delta_right_encoder_pos - delta_left_encoder_pos) / L;
        double phi = WrapRads(Last_yaw - yaw);
        telemetry.addData("phi", phi);
        double delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2.0;
        double delta_perp_pos = delta_center_encoder_pos - B * phi;

        double delta_x = delta_perp_pos * Math.cos(heading) - delta_middle_pos * Math.sin(heading);
        double delta_y = delta_perp_pos * Math.sin(heading) + delta_middle_pos * Math.cos(heading);

        Posx += delta_x;
        Posy += delta_y;
//        heading += phi;

//        heading = WrapRads(heading);

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;
        prev_center_encoder_pos = center_encoder_pos;
        Last_yaw = yaw;

    }

    public void move(double tilex, double tiley, double setpoint, double[] basespeed, double[] Kpidf_R,
                     double[] Kpidf_X, double[] Kpidf_Y, double Brake_Time, double height) {
        Controller  pidR    = new Controller(Kpidf_R[0], Kpidf_R[1], Kpidf_R[2], Kpidf_R[3], basespeed[0], toRadian(0.75));
        Controller  DelthaX = new Controller(Kpidf_X[0], Kpidf_X[1], Kpidf_X[2], Kpidf_X[3], basespeed[1], 2);
        Controller  DelthaY = new Controller(Kpidf_Y[0], Kpidf_Y[1], Kpidf_Y[2], Kpidf_Y[3], basespeed[2], 2);
        double targetx = tilex * tileSize[0];
        double targety = tiley * tileSize[1];
        int IS_Complete = 0;
        this.Current_Time = System.nanoTime() * 1E-9;
        this.Last_Time = this.Current_Time;
        while (opModeIsActive()) {
            this.Current_Time = System.nanoTime() * 1E-9;
            Odomentry();
//            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double Vx = DelthaX.Calculate((targetx - Posx)*-1);
            double Vy = DelthaY.Calculate(targety - Posy);

            double x2    =  (Math.cos(heading) * Vx) - (Math.sin(heading) * Vy);
            double y2    =  (Math.sin(heading) * Vx) + (Math.cos(heading) * Vy);

            double r =  pidR.Calculate(WrapRads(toRadian(setpoint) - heading));
            double d = Math.max(Math.abs(Vx) + Math.abs(Vy) + Math.abs(r), 1);
            double Move_Factor = Range.clip(this.Current_Time - this.Last_Time, 0, 1);
            MovePower((y2 - x2 - r) / d * Move_Factor, (y2 + x2 + r) / d * Move_Factor,
                    (y2 + x2 - r) / d * Move_Factor, (y2 - x2 + r) / d * Move_Factor);
            double  curPos     = Math.max(LL.getCurrentPosition(), RL.getCurrentPosition());
            double  Lift_Power = (curPos < (height + 100) && curPos > (height - 100)) ? 0 : curPos > height ? -1 : 1;

            LiftPower(Lift_Power);

            if (curPos > 50) {
                SetServoPos(0.08, LA, RA);
                SetServoPos(0.68, LJ, RJ);
                SetServoPos(0, Claw);
                continue;
            }

            if (curPos < 50) {
                SetServoPos(0, LA, RA);
                SetServoPos(1, LJ, RJ);
                continue;
            }

            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("Move_Factor", Move_Factor);
            telemetry.addData("XY", "%6f cm %6f cm" , Posx, Posy);
            telemetry.addData("tagetXtargetY", "%6f cm %6f cm" , targetx, targety);
            telemetry.addData("R", "%6f cm/s %6f cm" , r,  pidR.ErrorTolerance);
            telemetry.addData("X", "%6f cm/s %6f cm" , Vx, DelthaX.ErrorTolerance);
            telemetry.addData("Y", "%6f cm/s %6f cm" , Vy, DelthaY.ErrorTolerance);
            telemetry.addData("ErrorR", pidR.Error);
            telemetry.addData("ErrorX", DelthaX.Error);
            telemetry.addData("ErrorY", DelthaY.Error);
            telemetry.addData("Complete", IS_Complete);
            telemetry.addData("lift", curPos);
            telemetry.update();
            if (Vx <= 0.1 && Vy <= 0.1 && r <= 0 && AtTargetRange(curPos, height, 10)) {
                IS_Complete += 1;
                if (IS_Complete > 1) break;
                continue;
            }
            IS_Complete = 0;
        }
        Break(Brake_Time);
    }


    public void MovePower(double Front_Left, double Front_Right,
                          double Back_Left,  double Back_Right) {
        FL.setPower(Front_Left);
        FR.setPower(Front_Right);
        BL.setPower(Back_Left);
        BR.setPower(Back_Right);
    }
    public void Break(double stopSecond) {
        if (stopSecond == 0) return;
        MovePower(0, 0, 0, 0);
        MoveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep((long) (stopSecond * 1000));
    }

    public void MoveMode(DcMotor.RunMode moveMode) {
        FL.setMode(moveMode);
        FR.setMode(moveMode);
        BL.setMode(moveMode);
        BR.setMode(moveMode);
    }

    public void LiftPower(double liftPower) {
        LL.setPower(liftPower);
        RL.setPower(liftPower);
    }

    public double SetServoPos(double pos, float[] minMax, Servo L_servo, Servo R_servo) {
        pos = Range.clip(pos, minMax[0], minMax[1]);
        L_servo.setPosition(pos);
        R_servo.setPosition(pos);
        return pos;
    }

    public double SetServoPos(double pos, Servo L_servo, Servo R_servo) {
        pos = Range.clip(pos, 0, 1);
        L_servo.setPosition(pos);
        R_servo.setPosition(pos);
        return pos;
    }

    public double SetServoPos(double pos, float[] minMax, Servo servo){
        pos = Range.clip(pos, minMax[0], minMax[1]);
        servo.setPosition(pos);
        return pos;
    }

    public double SetServoPos(double pos, Servo servo){
        pos = Range.clip(pos, 0, 1);
        servo.setPosition(pos);
        return pos;
    }
    public void Initialize(DcMotor.RunMode moveMode, double[] DuoServoAng, double[] ServoAng) {
        imu = hardwareMap.get(IMU.class,       "imu");
        FL  = hardwareMap.get(DcMotorEx.class, "Front_Left");    FR  = hardwareMap.get(DcMotorEx.class, "Front_Right");
        BL  = hardwareMap.get(DcMotorEx.class, "Back_Left");     BR  = hardwareMap.get(DcMotorEx.class, "Back_Right");
        LL  = hardwareMap.get(DcMotorEx.class, "Left_lift");     RL  = hardwareMap.get(DcMotorEx.class, "Right_lift");
        LA  = hardwareMap.get(Servo.class, "Left_arm");          RA  = hardwareMap.get(Servo.class, "Right_arm");
        RC  = hardwareMap.get(Servo.class, "Rotation_Claw");     Claw= hardwareMap.get(Servo.class, "Claw");
        Ll  = hardwareMap.get(Servo.class, "Left_link");         Rl  = hardwareMap.get(Servo.class, "Right_link");
        ADL = hardwareMap.get(Servo.class, "Adjust_left");       ADR = hardwareMap.get(Servo.class, "Adjust_right");
        RJ  = hardwareMap.get(Servo.class, "Right_joint");       LJ  = hardwareMap.get(Servo.class, "Left_joint");
        RS = hardwareMap.get(TouchSensor.class, "Right_touch");
        Last_yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        encoder1 = FL;
        encoder2 = FR;
        encoder3 = BL;

        // Initialize IMU
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection .RIGHT)));
        // Reverse Servo
        Rl.setDirection(Servo.Direction.REVERSE);
        LA.setDirection(Servo.Direction.REVERSE);
        LJ.setDirection(Servo.Direction.REVERSE);
        ADR.setDirection(Servo.Direction.REVERSE);
        RC.setDirection(Servo.Direction.REVERSE);
//        Claw.setDirection(Servo.Direction.REVERSE);
        // Set Servo Position
        SetServoPos(0, LA, RA);
        SetServoPos(0, Ll, Rl);
        SetServoPos(1, LJ, RJ);
        SetServoPos(0.5, ADL, ADR);
        SetServoPos(0.12, RC);
        SetServoPos(0, Claw);
        // setMode Motors
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MoveMode(moveMode);

        // Reverse Motors
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        LL.setDirection(DcMotorSimple.Direction.REVERSE);
        // SetBehavior Motors
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // SetPower Motors
        MovePower(0, 0, 0, 0);
        LL.setPower(0);
        RL.setPower(0);
    }
}