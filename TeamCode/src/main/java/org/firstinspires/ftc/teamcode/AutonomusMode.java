package org.firstinspires.ftc.teamcode;
//Blbliotecas nescessárias para o TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//Motor e afins
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private CRServo ServoC;                 // gatilho, servo de rotação contínua   // MUDOU
    private Servo servoTorreta;
    private double posicaoTorreta = 0.5;    // torreta começa no meio
    private ElapsedTime timer = new ElapsedTime();

    // Servos Modeladores (Se eles controlam posições, devem ser do tipo Servo)
    private Servo servoPotencia;
    private Servo servo2Potencia;
    private double PotenciaAtual = 1;
    private double PotenciaAtual2 = 1;

    // Odometria GoBilda Pinpoint
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        inicializarHardware();
        invmotores();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odometry();   // configura a odometria UMA vez, antes do start

        telemetry.addData("Status", "Pronto");
        telemetry.addData("ServoC", ServoC == null ? "NAO ENCONTRADO na config" : "ok");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            odo.update();   // atualiza a odometria a cada ciclo

            movimentacao();
            coletor();
            motorsLancador();
            servoLancador();
            servoTORRETA();

            Pose2D pos = odo.getPosition();

            telemetry.addData("Torreta", "%.2f", posicaoTorreta);
            telemetry.addData("Lancador", "%.2f", motorPotencia.getPower());
            telemetry.addData("ServoC", ServoC == null ? "NAO ENCONTRADO" : "ok");
            telemetry.addData("Auxilio", "%.2f", Auxilio.getPower());   // MUDOU
            telemetry.update();
        }
    }

    public void invmotores() {
        Lef.setDirection(DcMotor.Direction.FORWARD);
        Let.setDirection(DcMotor.Direction.FORWARD);
        Ldf.setDirection(DcMotor.Direction.REVERSE);
        Ldt.setDirection(DcMotor.Direction.REVERSE);
        Coletor.setDirection(DcMotor.Direction.FORWARD);
        Auxilio.setDirection(DcMotor.Direction.REVERSE);
    }

    // Controle 1 - Locomoção
    public void movimentacao() {
        double  axial   =  this.gamepad1.left_stick_y,
                lateral = -this.gamepad1.left_stick_x,
                rot     = -this.gamepad1.right_stick_x;

        double VLef = axial - lateral + rot,
                VLdf = axial + lateral - rot,
                VLet = axial + lateral + rot,
                VLdt = axial - lateral - rot;

        double max = Math.max(Math.abs(VLef), Math.abs(VLdf));
        max = Math.max(max, Math.abs(VLet));
        max = Math.max(max, Math.abs(VLdt));

        if (max > 0.75) {
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
        }
        botaoAAnterior = botaoAAtual;
        if (coletorLigado) {
            Coletor.setPower(1.0);
            Auxilio.setPower(1.0);
        } else {
            Coletor.setPower(0.0);
            Auxilio.setPower(0.0);
        }

    }

    // Controle 2 - Lançador (analógico direito pra cima) e gatilho (Y)
    public void motorsLancador() {
        double y = -gamepad2.right_stick_y;        // pra cima = positivo
        if (y < 0.03) y = 0;                       // perto do centro ou pra baixo = desligado
        motorPotencia.setPower(y * 0.57);          // analógico no topo = 0.57

        // Gatilho (ServoC): segura Y pra girar, solta pra parar   // MUDOU
        if (ServoC != null) {
            if (gamepad2.y) {
                ServoC.setPower(1.0);
            } else {
                ServoC.setPower(0.0);
            }
        }
    }

    // Controle 2 - Torreta (analógico esquerdo pros lados)
    public void servoTORRETA() {     //*******
        double tempo = timer.seconds();            // segundos desde o último ciclo
        timer.reset();

        double x = gamepad2.left_stick_x;          // -1 = esquerda, 1 = direita
        if (Math.abs(x) < 0.1) x = 0;              // perto do centro conta como solto
        posicaoTorreta = posicaoTorreta + x * 0.5 * tempo;   // 0.5 = velocidade da torreta
        if (posicaoTorreta > 0.7) posicaoTorreta = 0.7;      // limite da estrutura
        if (posicaoTorreta < 0.0) posicaoTorreta = 0.0;
        servoTorreta.setPosition(posicaoTorreta);
    }

    //Servos Auxiliadores
    public void servoLancador() {
        if (gamepad2.a) {
            PotenciaAtual  = servoPotencia.getPosition();
            PotenciaAtual2 = servo2Potencia.getPosition();

            servoPotencia.setPosition(PotenciaAtual + 1);

            servo2Potencia.setPosition(PotenciaAtual2 + 1);
        }
        if (gamepad2.x) {
            servoPotencia.setPosition(PotenciaAtual);

            servo2Potencia.setPosition(PotenciaAtual2);
        }

    }

    public void odometry(){

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

    public void inicializarHardware() {
        Ldf            = hardwareMap.get(DcMotor.class,   "frontRight");
        Ldt            = hardwareMap.get(DcMotor.class,   "backRight");
        Lef            = hardwareMap.get(DcMotor.class,   "frontLeft");
        Let            = hardwareMap.get(DcMotor.class,   "backLeft");
        Coletor        = hardwareMap.get(DcMotor.class,   "motorExtra");
        Auxilio        = hardwareMap.get(DcMotor.class,   "motorAuxiliar");
        motorPotencia  = hardwareMap.get(DcMotor.class,   "motorPotencia");
        servoTorreta   = hardwareMap.get(Servo.class,     "servoTorreta");
        servoPotencia  = hardwareMap.get(Servo.class,     "servoPotencia");
        servo2Potencia = hardwareMap.get(Servo.class,     "servo2Potencia");
        ServoC         = hardwareMap.get(CRServo.class,   "ServoColetor");
        odo            = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        motorPotencia.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servoPotencia.setDirection(Servo.Direction.FORWARD);
        servo2Potencia.setDirection(Servo.Direction.REVERSE);

    }
}