package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;
import org.firstinspires.ftc.teamcode.Functions.MoveAutocorrect2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Functions.DataLogger.DataLoggerText;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;

import java.io.IOException;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions.RotateMove;
import org.firstinspires.ftc.teamcode.Functions.Unused.XYSystem.SistemXY;
import org.firstinspires.ftc.teamcode.Functions.MV.MVTurnTowardsPoint;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;
import org.firstinspires.ftc.teamcode.Functions.Arm;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;

import org.firstinspires.ftc.teamcode.Functions.Unused.XYSystem.PositionCalculator;


@TeleOp(name="_test", group="TEST")
@Disabled
// Facut de Vlad
public class Main extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //declarare motoare,servo,senzori:
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack, vaccumMotor,/* left_pistol*/right_pistol, arm_motor;
    public ModernRoboticsI2cRangeSensor rangeFinder = null;
    private Servo push_ring, hold_wobble;
    private double power;
    public Move move;
    public Rotate rotate;
    public Aspirator aspirator;
    public Pistol pistol;
    public VoltageReader voltageReader;
    public Arm arm;
    public HoldWobble holdwobble;
    public RotationDetector rotationDetector;
    public Tragaci tragaci;
    double poz;
    public MoveAutocorrect2 moveAuto;
    public PositionCalculator positionCalculator;
    public SistemXY sistemXY;
    public DataLogger dataLogger;
    public MVTurnTowardsPoint MVTurnTowardsPoint;
    private DataLoggerText dataLoggerText;
    private RotateMove rotateMove;

    @Override
    public void init() {
        //initializare motoare,servo,senzori:
        telemetry.addData("Time", runtime);

        telemetry.addData("Status", "Initialized");
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        push_ring = hardwareMap.servo.get("PR");
        right_pistol = hardwareMap.dcMotor.get("RP");
        arm_motor= hardwareMap.dcMotor.get("AM");
        vaccumMotor= hardwareMap.dcMotor.get("AM");


        hold_wobble = hardwareMap.servo.get("HW");
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);

        aspirator = new Aspirator(vaccumMotor);

       // brat = new Brat(arm_motor);
        holdwobble = new HoldWobble(hold_wobble);
        tragaci = new Tragaci(push_ring);

        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        rotateMove = new RotateMove(leftMotor, rightMotor, leftMotorBack, rightMotorBack, hardwareMap.get(BNO055IMU.class, "imu"));

        VoltageSensor VS =  this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        pistol = new Pistol( right_pistol, voltageReader);
        moveAuto = new MoveAutocorrect2(rotationDetector, move, rotate);
        arm_motor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Starting voltage", voltageReader.ReturnVoltage());

        try {
            positionCalculator = new PositionCalculator(move, 0, 0, rotationDetector, voltageReader);
        } catch (IOException e) {
            e.printStackTrace();
        }
        //dataLogger = new DataLogger(rotationDetector, VS, pistol, move, positionCalculator, getClass().getName());

        MVTurnTowardsPoint = new MVTurnTowardsPoint();
        sistemXY = new SistemXY(positionCalculator, MVTurnTowardsPoint, rotationDetector, move, rotate, telemetry);
        dataLoggerText = new DataLoggerText(rotationDetector, VS, pistol, move, sistemXY, positionCalculator, getClass().getName());
        // astea is doar de test nu le sterge inca
        telemetry.addData("Test 360", voltageReader.GetWaitTime(360, 3));
        telemetry.addData("Starting voltage", voltageReader.ReturnVoltage());
        telemetry.addData("CurrentTime2", voltageReader.CurrentTimeRead(5));
        telemetry.addData("CurrentVoltage2", voltageReader.CurrentTimeRead(6));

        telemetry.addData("ReturnRotation: ", rotationDetector.ReturnRotation());
        //holdwobble.Start();

    }


    @Override
    public void init_loop() {
        positionCalculator.CalibrateAccel();
    }



    @Override
    public void start() {
        runtime.reset();
        tragaci.Stop(); //cand se initializeaza robotul servoul este setat la pozitia 0.4


        // vaccumServoRight.setPosition(0);
    }


    @Override
    public void loop() {
        telemetry.addData("Return Voltage:", voltageReader.ReturnVoltage());

        if(gamepad1.dpad_up) //merge in fata
        {
            //moveAuto.SaveStartAngle();
            if(gamepad1.dpad_left){
                move.MoveOne(1, -1);
                move.MoveOne(4, 1);
            }
            else if(gamepad1.dpad_right){
                move.MoveOne(2, -1);
                move.MoveOne(3, 1);
            }
            else{
                move.MoveFull(1);
            }
            TelemetryData();
        }
        else if(gamepad1.dpad_down) //merge in spate
        {
            //moveAuto.SaveStartAngle();
            move.MoveFull(2);
            //positionCalculator.Update(-1);
            TelemetryData();
        }
        else if(gamepad1.dpad_left) //merge stanga
        {
            rotate.RotateFull(1);

        }
        else if(gamepad1.dpad_right) // merge dreapta
        {
            rotate.RotateFull(2);
        }
        if(!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.right_bumper && !gamepad1.left_bumper
                && gamepad1.left_stick_x==0 && gamepad1.left_stick_y==0 && gamepad1.right_stick_x==0 && !gamepad1.a)
        {
           // rotateMove.StartNewMovement();
            move.MoveStop();
        }
        //miscarea stanga-dreapta:
        if(gamepad1.right_bumper)
        {
            //moveAuto.SaveStartAngle();
            move.MoveFull(3);
            positionCalculator.NewUpdate(getRuntime());
        }
        if(gamepad1.left_bumper)
        {

            //moveAuto.SaveStartAngle();
            move.MoveFull(4);
            positionCalculator.NewUpdate(getRuntime());
        }
        //se termina codul pentru miscarea stanga-dreapta;

        //se termina codul pentru miscarea robotului;


        if(gamepad1.a) //porneste ambele motoare pentru luarea ring-urilor;
        {
            //aspirator.SwitchAndWait(1, runtime.seconds());
        }
        if(gamepad1.b) //pornesc motoarele de la pistol;
        {
            pistol.SwitchAndWait(1, runtime.seconds());
        }
        if(gamepad1.x) //atata timp cat butonul x este apasat servo-ul care impinge ring-urile este la pozitia 0.4
        {
            tragaci.SwitchAndWait(0.2,getRuntime());
            //tragaci.Start();
            //push_ring.setPosition(0.43);
        }
        if(!gamepad1.x)
        {
            tragaci.Stop();
            //push_ring.setPosition(0.6);
        }
        if(gamepad1.y)
        {
            aspirator.SwitchAndWaitInv(0.2, runtime.seconds());
        }
        if(gamepad1.right_trigger!=0)
        {
            //brat.StartInvers();
        }
        else if(gamepad1.left_trigger!=0)
        {
            //brat.Start();
        }
        else if(gamepad1.left_trigger==0 && gamepad1.right_trigger==0)
        {
            arm.Stop();
        }
        if(gamepad1.right_stick_button)
        {
            holdwobble.SwitchAndWait(1, runtime.seconds());

        }

        if(gamepad1.right_stick_x!=0){
            if(gamepad1.right_stick_x>0)
            {
                rotate.RotateRaw(2, gamepad1.left_stick_x);

            }
            else if(gamepad1.right_stick_x<0)
            {
                rotate.RotateRaw(1, gamepad1.left_stick_x);
            }
            else rotate.RotateStop();

        }
        else{
            if(gamepad1.left_stick_x!=0){
                move.MoveRaw(3, gamepad1.left_stick_x);
            }
            if(gamepad1.left_stick_y!=0){
                move.MoveRaw(1, -gamepad1.left_stick_y);
            }
        }

        if(aux>=1)
        {
            aux=1;
        }
        else if(aux<=0.4){
            aux=0.4;
        }

        if(gamepad1.right_stick_y!=0){
            if(gamepad1.right_stick_y>0){
                aux+=0.01;
            }
            else{
                aux-=0.01;
            }
            pistol.StartRaw(-aux);
        }

        if(gamepad1.a){
            sistemXY.GoTo(0.2, 0.2);
            TelemetryData();
        }

        TelemetryData();

    }
    double aux=1;

    void TelemetryData(){
        telemetry.addLine(rotateMove.DebugData());
        telemetry.update();
    }


    @Override
    public void stop() {
        //holdwobble.Stop();
        //dataLogger.CloseFile();
    }

}
