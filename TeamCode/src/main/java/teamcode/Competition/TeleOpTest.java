package teamcode.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Point;
import teamcode.common.Vector2D;

@Disabled
@TeleOp(name = "FirstTeleOpScript")
public class TeleOpTest extends AbstractOpMode {

    private final double SPRINT_LINEAR_MODIFIER = 0.75;
    private final double NORMAL_LINEAR_MODIFIER = 0.5;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.68;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double INTAKE_MOTOR_POWER = 0.6;

    MecanumDriveTrain driveTrain;
    Shooter shooter;
    Localizer localizer;
    Thread driveThread;
    Thread shootThread;



    @Override
    protected void onInitialize() {
    // try-catch stuff with localizer and drive train
        try {
            Scanner fileParser = new Scanner(new File(Constants.SAVE_FILE_PATH));
            localizer = new Localizer(hardwareMap, new Vector2D(fileParser.nextDouble(), fileParser.nextDouble()), fileParser.nextDouble());
            driveTrain = new MecanumDriveTrain(hardwareMap);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        driveThread = new Thread(){
            public void run(){
                while(opModeIsActive()) {
                    /*
                    If the left bumper is either pressed or held (unsure), the 'sprint' modifiers are used to move faster. Otherwise, speed is normal.
                    */
                    telemetry.addData("Mode", "running");
                    telemetry.update();

                    if(gamepad1.left_bumper){
                        driveTrain.setPower(new Vector2D( gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER),
                                gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER);

                    }else{
                        driveTrain.setPower(new Vector2D( gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER),
                                gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER);
                    }

                }
            }
        };



        shootThread = new Thread(){
            public void run(){
                while(opModeIsActive()){
                    //do a variety of conditional things to manipulate scoring
                    //ex:
                    /*
                        - When the left trigger is pushed past a certain limit the intake fires up.
                        - When the x button is pressed, the shooter does it's thing (the assumption is made that localizer is being used for autocorrection).
                    */

                    if(gamepad1.left_trigger > 0.3){
                        shooter.intake(INTAKE_MOTOR_POWER);
                     }
                    if(gamepad1.x) {
                        shooter.shoot(1, 0.95);
                }
            }
        }
    };

    }

    @Override
    protected void onStart() {
        shootThread.start();
        driveThread.start();
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {
        telemetry.addData("Mode", "stopped");
        telemetry.update();

        driveTrain.setPower(0.0,0.0,0.0,0.0);


    }


}
