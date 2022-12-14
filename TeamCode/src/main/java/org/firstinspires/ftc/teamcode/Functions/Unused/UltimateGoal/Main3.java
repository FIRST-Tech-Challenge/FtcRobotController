package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;
import org.firstinspires.ftc.teamcode.Functions.Arm;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;

import java.io.IOException;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions.MV.MVTurnTowardsPoint;
import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;

import org.firstinspires.ftc.teamcode.Functions.Unused.XYSystem.PositionCalculator;

import java.lang.Math;


@TeleOp(name="Main3", group="MAIN")
@Disabled
// Facut de Vlad
public class Main3 extends OpMode
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
    //public MoveAutocorrectTest moveAuto;
    public PositionCalculator positionCalculator;
    public DataLogger dataLogger;
    public MVTurnTowardsPoint MVTurnTowardsPoint;

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


        VoltageSensor VS =  this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        pistol = new Pistol( right_pistol, voltageReader);
        //moveAuto = new MoveAutocorrectTest(rotationDetector, move);
        arm_motor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Starting voltage", voltageReader.ReturnVoltage());

        try {
            positionCalculator = new PositionCalculator(move, 0, 0, rotationDetector, voltageReader);
        } catch (IOException e) {
            e.printStackTrace();
        }
        //dataLogger = new DataLogger(rotationDetector, VS, pistol, move, positionCalculator, getClass().getName());
        MVTurnTowardsPoint = new MVTurnTowardsPoint();
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

    }



    @Override
    public void start() {
        runtime.reset();

        //push_ring.setPosition(0.62); //cand se initializeaza robotul servoul este setat la pozitia 0.4
        tragaci.Stop();

        // vaccumServoRight.setPosition(0);
    }

    int Stage=1;
    @Override
    public void loop() {
        telemetry.addData("Return Voltage:", voltageReader.ReturnVoltage());

        if(gamepad2.dpad_up) //merge in fata
        {
            move.MoveFull(1);
            TelemetryData();
        }
        else if(gamepad2.dpad_down) //merge in spate
        {
            move.MoveFull(2);
            TelemetryData();
        }
        else if(gamepad2.dpad_left) //merge stanga
        {
            rotate.RotateFull(1);

        }
        else if(gamepad2.dpad_right) // merge dreapta
        {
            rotate.RotateFull(2);
        }
        //miscarea stanga-dreapta:
        else if(gamepad2.right_bumper)
        {
            //moveAuto.SaveStartAngle();
            move.MoveFull(3);
        }
        else if(gamepad2.left_bumper)
        {
            move.MoveFull(4);
        }
        else if(gamepad2.right_stick_x!=0){
            rotate.RotateRaw(1, -gamepad2.right_stick_x/2);

        }
        else if(gamepad2.left_stick_y!=0){
            move.MoveRaw(1, -gamepad2.left_stick_y);
        }
        if(!gamepad2.dpad_down && !gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.dpad_up && !gamepad2.right_bumper && !gamepad2.left_bumper
                && gamepad2.left_stick_x==0 && gamepad2.left_stick_y==0 && gamepad2.right_stick_x==0 && !gamepad2.a && !gamepad2.b && gamepad2.right_stick_y==0)
        {
            move.MoveStop();
        }

        if(gamepad2.a){
            int Unghi = 0;
            if(rotationDetector.WaitForRotation(Unghi))
            {
                rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
            }

        }

        if(!gamepad2.b){
            Stage=1;
        }
        if(gamepad2.b){
            Stage=GoTo(new MVVariables.Vector2(0, 0), Stage);
        }

        if(gamepad2.x){
            positionCalculator.Position = new MVVariables.Vector2(0, 0);
            telemetry.addLine("Firing position reset");
        }
        //se termina codul pentru miscarea stanga-dreapta;

        //se termina codul pentru miscarea robotului;


        if(gamepad1.a) //porneste ambele motoare pentru luarea ring-urilor;
        {
            aspirator.SwitchAndWait(1, runtime.seconds());
        }
        if(gamepad1.b) //pornesc motoarele de la pistol;
        {
            pistol.SwitchAndWait(1, runtime.seconds());
            if(aspirator.CheckStatus()){
                aspirator.Stop();
            }
        }
        if(gamepad1.x) //atata timp cat butonul x este apasat servo-ul care impinge ring-urile este la pozitia 0.4
        {
            tragaci.SwitchAndWait(0.4,getRuntime());
            positionCalculator.Position = new MVVariables.Vector2(0, 0);
            telemetry.addLine("Firing position reset");
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
        if(gamepad2.y)
        {
            holdwobble.SwitchAndWait(1, runtime.seconds());

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


        TelemetryData();

    }
    double aux=1;

    void TelemetryData(){
        telemetry.addData("Pistol power", pistol.ReturnMotorSpeed());
        if(getRuntime()==(int)getRuntime()) {
            dataLogger.writeData(getRuntime());
        }
        telemetry.addLine(positionCalculator.NewUpdate(getRuntime()));

        telemetry.update();
    }


    int GoTo(MVVariables.Vector2 Destination, int Stage){

        int Unghi = (int) MVTurnTowardsPoint.AngleCalculator(Destination, positionCalculator.Position, rotationDetector.ReturnPositiveRotation());
        if(rotationDetector.WaitForRotation(Unghi)&&Stage<=1)
        {
            telemetry.addLine("Unghi:"+Unghi);
            TelemetryData();
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
            return 1;
        }
        // aceasta linie e doar de siguranta.
        // verificam care distanta e mai mare; spre x sau spre y si doar atunci cand atinge distanta aia se va opri

        else if((Math.abs(Destination.magnitude - positionCalculator.Position.magnitude)) >= 0.005){
            if((Math.abs(Destination.magnitude - positionCalculator.Position.magnitude)) >= 0.01) {
                move.MoveFull(1);
                TelemetryData();
                return 2;

            }
            else{
                move.MoveRaw(1, 0.2);
                TelemetryData();
                return 3;

            }

        }

        move.MoveStop();
        return 0;
    }

    @Override
    public void stop() {
        holdwobble.Stop();
    }

}
