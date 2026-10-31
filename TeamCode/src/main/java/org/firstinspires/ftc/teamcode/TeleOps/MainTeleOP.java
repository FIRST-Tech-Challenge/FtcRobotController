import org.firstinspires.ftc.teamcode.subsystems.LightSubsystem;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode 

{private LightSubsystem lights;

@Override
public void runOpMode() 

}

    lights = new LightSubsystem(hardwareMap);

    waitForStart();

   while (opModeIsActive()) {

            // Update the light
            lights.update(shooterOn, shooterReady);
   }