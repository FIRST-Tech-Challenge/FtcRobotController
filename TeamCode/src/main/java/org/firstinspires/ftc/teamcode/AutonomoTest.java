package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutonomeTest", group = "Robot")
@Disabled
public class AutonomoTest extends LinearOpMode {

    private DcMotor FL0 = null;
    private DcMotor FR1 = null;
    private DcMotor BL2 = null;
    private DcMotor BR3 = null;
    private final ElapsedTime runtime = new ElapsedTime();
    static final double CONTS_PER_MORTOR_REV     = 28; // por exemplo: codificador de motor HD REX
    static final double DRIVE_GEAR_REDUCTION     = 20.0; // 20:1 engrenagens externas.
    static final double WHEEL_DIAMETER_INCHES    = 3.78; // Para calcular a circunferência roda preta 2.99 roda amarela 3.78
    static final double COUNTS_PER_INCH          = (CONTS_PER_MORTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED              = 0.7;
    static final double TURN_SPEED               = 0.7;

    @Override
    public void runOpMode(){

        // Inicializa as variáveis do sistema de acionamento.
        FL0 = hardwareMap.get(DcMotor.class,"FL0");
        FR1 = hardwareMap.get(DcMotor.class,"FR1");
        BL2 = hardwareMap.get(DcMotor.class,"BL2");
        BR3 = hardwareMap.get(DcMotor.class,"BR3");

        // Para avançar, a maioria dos robôs precisa que o motor de um lado esteja invertido, pois os eixos apontam em direções opostas.
        // Quando executado, este OpMode deve fazer com que ambos os motores avancem. Portanto, ajuste essas duas linhas com base no seu primeiro teste de direção.
        // Observação: As configurações aqui pressupõem acionamento direto nas rodas esquerda e direita. Redução de marcha ou acionamentos de 90 graus podem exigir inversões de direção.
        FL0.setDirection(DcMotor.Direction.FORWARD);
        FR1.setDirection(DcMotor.Direction.REVERSE);
        BL2.setDirection(DcMotor.Direction.FORWARD);
        BR3.setDirection(DcMotor.Direction.REVERSE);


        FL0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        FL0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Começando em", "FL:%7d FR:%7d BL:%7d BR:%7d",
                        FL0.getCurrentPosition(),
                        FR1.getCurrentPosition(),
                        BL2.getCurrentPosition(),
                        BR3.getCurrentPosition());
        telemetry.update();

        waitForStart();

        //Caminho a ser Realizado
        enconderDrive(DRIVE_SPEED, 50,50,50,50,5.0);
        enconderDrive(TURN_SPEED,-18,18,-18,18,5.0);
        enconderDrive(TURN_SPEED,18,-18,18,-18,5.0);
        enconderDrive(DRIVE_SPEED, -50,-50,-50,-50,5.0);
        telemetry.addData("Caminho", "Completo");
        telemetry.update();

    }
    /*
     * Metodo para realizar um movimento relativo, com base nas contagens do encoder.
     * Os encoders não são reiniciados, pois o movimento é baseado na posição atual.
     * O movimento será interrompido se ocorrer qualquer uma das três condições:
     * 1) O movimento atingir a posição desejada
     * 2) O tempo de movimento se esgotar
     * 3) O driver interrompe a execução do OpMode.
     */
    public void enconderDrive(double speed,
                              double FLInches,
                              double FRInches,
                              double BLInches,
                              double BRInches,
                              double timeoutS){
        int newFLTarget;
        int newFRTarget;
        int newBLTarget;
        int newBRTarget;

        // Certifique-se de que o OpMode ainda esteja ativo
        if (opModeIsActive()){

            // Determina a nova posição alvo e passa para o controlador do motor
            newFLTarget = FL0.getCurrentPosition() + (int)(FLInches * COUNTS_PER_INCH);
            newFRTarget = FR1.getCurrentPosition() + (int)(FRInches * COUNTS_PER_INCH);
            newBLTarget = BL2.getCurrentPosition() + (int)(BLInches * COUNTS_PER_INCH);
            newBRTarget = BR3.getCurrentPosition() + (int)(BRInches * COUNTS_PER_INCH);
            FL0.setTargetPosition(newFLTarget);
            FR1.setTargetPosition(newFRTarget);
            BL2.setTargetPosition(newBLTarget);
            BR3.setTargetPosition(newBRTarget);

            // Ativar RUN_TO_POSITION
            FL0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // redefine o tempo limite e inicia o movimento.
            runtime.reset();
            FL0.setPower(Math.abs(speed));
            FR1.setPower(Math.abs(speed));
            BL2.setPower(Math.abs(speed));
            BR3.setPower(Math.abs(speed));

            // continue o loop enquanto ainda estivermos ativos, houver tempo restante e ambos os motores estiverem funcionando.
            // Observação: usamos (isBusy() && isBusy()) no teste de loop, o que significa que quando QUALQUER motor atingir
            // sua posição alvo, o movimento será interrompido. Isso é "mais seguro" caso o robô
            // sempre encerre o movimento o mais rápido possível.
            // No entanto, se você precisar que AMBOS os motores concluam seus movimentos antes que o robô continue
            // para a próxima etapa, use (isBusy() || isBusy()) no teste de loop.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL0.isBusy() && FR1.isBusy() && BL2.isBusy() && BR3.isBusy())){

                telemetry.addData("Status", "Run time: " + runtime.toString());
                telemetry.addData("Correndo para", "FL:%7d FR:%7d BL:%7d BR:%7d", newFLTarget, newFRTarget, newBLTarget, newBRTarget);
                telemetry.addData("Atualmente em", " FL:%7d FR:%7d BL:%7d BR:%7d",
                        FL0.getCurrentPosition(), FR1.getCurrentPosition(), BL2.getCurrentPosition(), BR3.getCurrentPosition());
                telemetry.update();
            }

            //Parar Movimento;
            FL0.setPower(0);
            FR1.setPower(0);
            BL2.setPower(0);
            BR3.setPower(0);

            //Desligar RUN_TO_POSITION
            FL0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);

        }
    }
}
