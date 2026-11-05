package org.firstinspires.ftc.teamcode;
//Blbliotecas nescessárias para o TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//Motor e afins
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//Odometria
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "MainCode", group = "Linear")
public class TeleOpfinal extends LinearOpMode {

    // Locomoção
    private DcMotor Ldf, Ldt, Lef, Let;

    // Coletor
    private DcMotor Coletor;
    private DcMotor Auxilio;
    private boolean coletorLigado  = false;
    private boolean botaoAAnterior = false;

    // Lançador
    private DcMotor motorPotencia;
    private Servo ServoC;
    private Servo servoTorreta;

    // Servos Modeladores (Se eles controlam posições, devem ser do tipo Servo)
    private Servo servoPotencia;
    private Servo servo2Potencia;
    private double UltimaPotencia = 0;
    private double Ultima2Potencia = 0;

    // Odometria GoBilda Pinpoint
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        inicializarHardware();
        invmotores();

        telemetry.addData("Status", "Pronto");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            movimentacao();
            coletor();
            motorsLancador();
            servoLancador();
            servoTORRETA();
            telemetriafinal();
            initHardware();

            Pose2D pos = odo.getPosition();
        }
    }

    public void invmotores() {
        Lef.setDirection(DcMotor.Direction.REVERSE);
        Let.setDirection(DcMotor.Direction.REVERSE);
        Ldf.setDirection(DcMotor.Direction.FORWARD);
        Ldt.setDirection(DcMotor.Direction.FORWARD);
        Coletor.setDirection(DcMotor.Direction.FORWARD);
        Auxilio.setDirection(DcMotor.Direction.FORWARD);
    }

    // Controle 1 - Locomoção
    public void movimentacao() {
        double  axial   = -this.gamepad1.left_stick_y,
                lateral =  this.gamepad1.left_stick_x,
                rot     =  this.gamepad1.right_stick_x;

        double VLef = axial - lateral + rot,
                VLdf = axial + lateral - rot,
                VLet = axial + lateral + rot,
                VLdt = axial - lateral - rot;

        double max = Math.max(Math.abs(VLef), Math.abs(VLdf));
        max = Math.max(max, Math.abs(VLet));
        max = Math.max(max, Math.abs(VLdt));

        if (max > 1) {
            VLef /= max;
            VLdf /= max;
            VLet /= max;
            VLdt /= max;
        }

        Lef.setPower(VLef);
        Ldf.setPower(VLdf);
        Let.setPower(VLet);
        Ldt.setPower(VLdt);
    }

    // Controle 1 - Coletor (botão A)
    public void coletor() {             //******
        boolean botaoAAtual = this.gamepad1.a;

        if (botaoAAtual && !botaoAAnterior) {
            coletorLigado = !coletorLigado;
            servoRLigado  = !servoRLigado;
            servoLLigado  = !servoLLigado;
        }

        Coletor.setPower(coletorLigado ? 1.0 : 0.0);
        Auxilio.setPower(coletorLigado ? 1.0 : 0.0);
        FlorR.setPosition(servoRLigado ? 1.0 : 0.0);
        FlorL.setPosition(servoLLigado ? 1.0 : 0.0);

        botaoAAnterior = botaoAAtual;
    }

    public void motorsLancador() {

        if (gamepad2.right_trigger > 0.1) {
            motorPotencia.setPower(0.59);
        } else {
            motorPotencia.setPower(0.0);
        }

        // Servo Gatilho liga junto do lançador
        if (gamepad2.right_trigger > 0.1) {
            ServoC.setPosition(1.0);
        } else {
            ServoC.setPosition(0.0);
        }
    }

    public void servoTORRETA(){     //*******
        if(gamepad2.dpad_left){
            servoTorreta.setPosition(1.0);
        }
        else if (gamepad2.dpad_right){
            servoTorreta.setPosition(-1.0);
        }
        else {
            servoTorreta.setPosition(0.0);
        }
    }

    public void servoLancador() {
        if (gamepad1.a) {
            UltimaPotencia = this.PotenciaAtual;
            Ultima2Potencia = this.PotenciaAtual2;

            servoPotencia.setPosition(0.25);

            servo2Potencia.setPosition(0.25);
        }
        if (gamepad1.x) {
            servoPotencia.setPosition(PotenciaAtual);

            servo2Potencia.setPosition(PotenciaAtual2);
        }
    }

    public void initHardware(){

            // Offsets = posição dos pods em relação ao centro de giro do robô (mm)
            odo.setOffsets(30, 12, DistanceUnit.MM);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);

            // CORREÇÃO: Substituído resetPosAndIMU() por recalibrateIMU() e setPosition()
            odo.recalibrateIMU();
            sleep(300);

            // Posição inicial do robô no campo
            odo.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));

            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
    }

    public void inicializarHardware() {
        Ldf            = hardwareMap.get(DcMotor.class,   "frontRight");
        Ldt            = hardwareMap.get(DcMotor.class,   "backRight");
        Lef            = hardwareMap.get(DcMotor.class,   "frontLeft");
        Let            = hardwareMap.get(DcMotor.class,   "backLeft");
        Coletor        = hardwareMap.get(DcMotor.class,   "motorExtra");
        Auxilio        = hardwareMap.get(DcMotor.class,   "motorAuxiliar");
        ServoC         = hardwareMap.get(CRServo.class,   "ServoColetor");
        motorPotencia  = hardwareMap.get(DcMotor.class,   "motorPotencia");  //lancador
        servoTorreta   = hardwareMap.get(Servo.class,     "servoTorreta"); //girar a torreta
        servoPotencia  = hardwareMap.get(Servo.class,     "servoPotencia");
        servoPotencia.setDirection(Servo.Direction.FORWARD);
        servo2Potencia = hardwareMap.get(Servo.class,     "servo2Potencia");
        servo2Potencia.setDirection(Servo.Direction.FORWARD);
        odo            = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    }

    public void telemetriafinal() {
        telemetry.addData("Status odo",       odo.getDeviceStatus());
        telemetry.update();
    }


