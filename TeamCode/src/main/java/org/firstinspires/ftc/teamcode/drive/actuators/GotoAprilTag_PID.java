package org.firstinspires.ftc.teamcode.drive.actuators;

// Importa bibliotecas necessárias para usar a câmera Limelight e o hardware do FTC
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// Define que esse código é autônomo e aparece no Driver Hub como "AprilTag Align"
@Autonomous(name = "AprilTag Align", group = "Autonomous")
public class GotoAprilTag_PID extends LinearOpMode {

    // Motores do robô
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Câmera Limelight
    private Limelight3A limelight;

    // Servos do robô (mão, braço, etc)
    Servo rotate, garra, pleft, pright, lright, lleft;

    // Variáveis para controle PID
    private double previousTx = 0;
    private double integralTx = 0;

    @Override
    public void runOpMode() {
        // Liga os motores com os nomes usados no Robot Configuration
        frontLeft = hardwareMap.get(DcMotor.class, "odol");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "odor");
        backRight = hardwareMap.get(DcMotor.class, "odom");

        // Liga os servos com os nomes do Configuration
        lright = hardwareMap.get(Servo.class, "lright");
        lleft = hardwareMap.get(Servo.class, "lleft");
        rotate = hardwareMap.get(Servo.class, "rotate");
        garra = hardwareMap.get(Servo.class, "garra");
        pleft = hardwareMap.get(Servo.class, "pleft");
        pright = hardwareMap.get(Servo.class, "pright");

        // Inverte os motores da esquerda (para frente virar frente mesmo)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Configura e liga a câmera Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2); // usa o pipeline 2
        limelight.start();

        // Espera o botão PLAY no Driver Hub
        waitForStart();

        // Enquanto o modo estiver rodando
        while (opModeIsActive()) {
            // Pega os dados mais recentes da Limelight
            LLResult result = limelight.getLatestResult();

            // Se a câmera ver uma tag
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // distância para os lados
                double ty = result.getTy(); // altura da tag
                double ta = result.getTa(); // tamanho da tag

                // Define constantes do PID
                double kP = 0.015;
                double kI = 0.0002;
                double kD = 0.002;

                // Calcula correções para alinhar com a tag
                integralTx += tx;
                double derivativeTx = tx - previousTx;
                double turn = kP * tx + kI * integralTx + kD * derivativeTx;
                previousTx = tx;

                // Move para frente baseado na distância vertical (ty)
                double forward = (15 - ty) * 0.04;

                // Limita os valores de força
                turn = clip(turn, -0.4, 0.4);
                forward = clip(forward, -0.4, 0.4);

                // Faz o robô se mover
                driveMecanum(0, forward, turn);

                // Mostra os dados no Driver Hub
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("ta", ta);
                telemetry.update();

                // Para se estiver bem alinhado
                if (Math.abs(tx) < 1.0 && ty > 9 && ta > 1.8) {
                    break;
                }
            } else {
                // Se não ver a tag, para o robô
                driveMecanum(0, 0, 0);
                telemetry.addLine("Tag não detectada");
                telemetry.update();
            }
        }

        // Para o robô no final
        driveMecanum(0, 0, 0);
        limelight.stop();
    }

    // Função para mover com rodas mecanum
    private void driveMecanum(double strafe, double forward, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    // Limita valor entre mínimo e máximo
    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
