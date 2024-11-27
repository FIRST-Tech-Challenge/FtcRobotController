import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="BlueSidedroite")
public class BlueSidedroite extends LinearOpMode {
    private DcMotor motorA;
    private DcMotor motorB;

    private Servo coudeA;
    private Servo pince;


    @Override
    public void runOpMode() {
        Gamepad manette1 = this.gamepad1;
        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorB = hardwareMap.get(DcMotor.class, "motorB");
        coudeA = hardwareMap.get(Servo.class, "coudeA");
        pince = hardwareMap.get(Servo.class, "pince");
        // Put initialization blocks here

        motorA.setPower(0);
        motorB.setPower(0);

        waitForStart();
        telemetry.addData("z :", "mode autonome initialiser");
        telemetry.update();

        while (opModeIsActive()) {
                coudeA.setPosition(0);


                motorA.setPower(-0.2);
                motorB.setPower(0.2);//avancer pour pouvoir mettre le specimene sur la barre
                coudeA.setPosition(0.90);

                sleep(1000);


                sleep(500)
                ;
                motorA.setPower(0.2);
                motorB.setPower(0.2);

                sleep(1350);//tourner vers la droite pour aller prendre les bloc d'equipe


                motorA.setPower(0);
                motorB.setPower(0);



        }

    }

}
