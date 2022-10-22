package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Functions.AccelerationDetector;
import org.firstinspires.ftc.teamcode.Functions.Arm;
import org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal.AccelerationReader;
import org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal.Aspirator;
import org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal.DataLogger;
import org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal.HoldWobble;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal.Pistol;
import org.firstinspires.ftc.teamcode.Functions.Unused.XYSystem.PositionCalculator;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal.RotationDetector2;
import org.firstinspires.ftc.teamcode.Functions.Unused.XYSystem.SistemXY;
import org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal.Tragaci;
import org.firstinspires.ftc.teamcode.Functions.MV.MVTurnTowardsPoint;
import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;

import java.io.IOException;



@Autonomous(name = "AutonomieTestare", group = "Concept")
@Disabled
public class TestareAutonomie extends LinearOpMode {

    private boolean ok=false;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack, vaccumMotor,/* left_pistol*/right_pistol, arm_motor;
    public ModernRoboticsI2cRangeSensor rangeFinder = null;
    private Servo push_ring, hold_wobble;
    private double power;
    public String label=null;
    public Move move;
    public Rotate rotate;
    public Aspirator aspirator;
    public Pistol pistol;
    public VoltageReader voltageReader;
    public Arm arm;
    public HoldWobble holdwobble;
    public RotationDetector rotationDetector;
    public RotationDetector2 rotationDetector2;
    public Tragaci tragaci;
    double poz;
     //public MoveAutocorrectTest moveAuto;
    public PositionCalculator positionCalculator;
    public AccelerationReader accelerationReader;
    public AccelerationDetector accelerationDetector;
    public DataLogger dataLogger;
    public SistemXY sistemXY;
    public MVTurnTowardsPoint MVTurnTowardsPoint;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Acum testam sa vedem daca merge 100 de cm in fata");
        //telemetry.addLine("Dupa se va roti la 270 de grate apoi la 90");
        telemetry.addLine("Daca testele astea sunt de succes, urmeaza sa comentezi de la linia la 130 pana la 150");
        telemetry.addLine("Si sa decomentezi de la linia 153-160");
        telemetry.update();
        vaccumMotor = hardwareMap.dcMotor.get("VM");
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        push_ring = hardwareMap.servo.get("PR");
        right_pistol = hardwareMap.dcMotor.get("RP");
       // DcMotor arm_motor = hardwareMap.dcMotor.get("AM");


        Servo hold_wobble = hardwareMap.servo.get("HW");
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        aspirator = new Aspirator(vaccumMotor);

      //  Brat brat = new Brat(arm_motor_left, arm_motor_right);
        HoldWobble holdwobble = new HoldWobble(hold_wobble);
        Tragaci tragaci = new Tragaci(push_ring);

        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));


        VoltageSensor VS = this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        pistol = new Pistol(right_pistol, voltageReader);
        //MoveAutocorrectTest moveAuto = new MoveAutocorrectTest(rotationDetector, move);
        arm_motor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Starting voltage", voltageReader.ReturnVoltage());

        try {
            positionCalculator = new PositionCalculator(move, 0, 0, rotationDetector, voltageReader);
        } catch (IOException e) {
            telemetry.addLine(e.toString());
        }
        dataLogger = new DataLogger(rotationDetector, VS, pistol, move, positionCalculator, getClass().getName());
        try {
            accelerationDetector = new AccelerationDetector(rotationDetector.ReturnGyro());
            accelerationReader = new AccelerationReader(rotationDetector.ReturnGyro(), move);
        } catch (IOException e) {
            telemetry.addLine(e.toString());
        }
        MVTurnTowardsPoint = new MVTurnTowardsPoint();
        sistemXY = new SistemXY(positionCalculator, MVTurnTowardsPoint, rotationDetector, move, rotate, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        while(!isStarted()){
            telemetry.addLine(positionCalculator.CalibrateAccel());
            telemetry.update();
        }
        sistemXY.Reset();


        waitForStart();


       /* long waitTime = voltageReader.GetWaitTime(100, 1);
        for(int i=0; i<waitTime;i++){
            move.MoveFull(2);
            telemetry.addLine(positionCalculator.NewUpdate(getRuntime()));
            sleep(1);
        }


        GoTo(new Variables.Vector2(0,0.5));


        int Unghi = 180;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            telemetry.addLine("Unghi:"+Unghi);
            TelemetryData();

            rotate.RotateRaw(1, rotationDetector.VitezaMotoare(Unghi));
        }*/
        int Unghi;
        Unghi=0;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        sleep(1000);
        Unghi=90;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1,rotationDetector.MotorPower(Unghi));
        }
        rotate.RotateStop();
        sleep(1000);
        Unghi=180;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        sleep(1000);
        Unghi=270;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();
        sleep(1000);
        Unghi=360;
        while(rotationDetector.WaitForRotation(Unghi))
        {
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop();

        /*
        sistemXY.Reset();
        TelemetryData();
        while(sistemXY.GoTo(0.5, 0.2)){
            TelemetryData();
            sleep(30);
        }

        sistemXY.SetDestination(0.5, 0.2);
        sistemXY.GivePosition(positionCalculator.Position);
        telemetry.addLine("Generated angle: " + sistemXY.GenerateAngle());
        telemetry.update();


        if (!sistemXY.IsX()){
            while (sistemXY.RotateToDestination()) {
                TelemetryData();
                sleep(1);
            }
        while (sistemXY.GoTowardsDestinationX()) {
            TelemetryData();
            sleep(1);
            }
        }

        if (!sistemXY.IsY()) {
            while (sistemXY.RotateToDestination()) {
                TelemetryData();
                sleep(1);
            }
            while (sistemXY.GoTowardsDestinationY()) {
                TelemetryData();
                sleep(1);
            }
        }
        TelemetryData();
        sleep(10000);

        // testare accelerationReader
        // trebuie sa mearga in fata exact 100 de cm
        /*


        accelerationReader.Reset(1, 100);
        do
        {
            move.MoveFull(1);
            telemetry.addLine(""+accelerationReader.ReturnData());
            sleep(accelerationReader.Go());

        }while(accelerationReader.IsNotDone());
        move.MoveStop();
        */

        telemetry.addLine("Test complete"); sleep(10000);




    }
    void TelemetryData(){
        telemetry.addLine("Current Angle: "+ sistemXY.GenerateAngle());
        sistemXY.GivePosition(positionCalculator.Position);
        dataLogger.writeData(getRuntime());
        telemetry.addLine(accelerationReader.ReturnData());
        telemetry.addLine(positionCalculator.NewUpdate(getRuntime()));
        telemetry.addLine("Current Rotation: "+ rotationDetector.ReturnPositiveRotation());
        telemetry.update();
    }

    void GoTo(MVVariables.Vector2 Destinatie){
        int Unghi = (int) MVTurnTowardsPoint.AngleCalculator(Destinatie, positionCalculator.Position, rotationDetector.ReturnPositiveRotation());
        while(rotationDetector.WaitForRotation(Unghi))
        {
            telemetry.addLine("Unghi:"+Unghi);
            TelemetryData();
            rotate.RotateRaw(1, rotationDetector.MotorPower(Unghi));
        }
        rotate.MoveStop(); // aceasta linie e doar de siguranta.
        // verificam care distanta e mai mare; spre x sau spre y si doar atunci cand atinge distanta aia se va opri

        while((Math.abs(Destinatie.magnitude - positionCalculator.Position.magnitude)) >= 0.01){
            if((Math.abs(Destinatie.magnitude - positionCalculator.Position.magnitude)) >= 0.025) {
                move.MoveFull(1);
                TelemetryData();

            }
            else{
                move.MoveRaw(1, 0.2);
                TelemetryData();

            }

        }
        move.MoveStop();
    }
}
