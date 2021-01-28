package teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import teamcode.League1.Shooter;
import teamcode.common.AbstractOpMode;
import teamcode.common.Constants;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Point;
import teamcode.common.Vector2D;

@TeleOp(name="exampleTeleOp")
public class SampleTeleOp extends AbstractOpMode {

    private final double SPRINT_LINEAR_MODIFIER = 0.75;
    private final double NORMAL_LINEAR_MODIFIER = 0.5;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.68;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double INTAKE_MOTOR_POWER = 0.6;


    MecanumDriveTrain driveTrain;
    Localizer localizer;
    Shooter shooter;
    Thread driveThread;
    Thread armThread;

    /*
    the reason we care about the localizer here is that if the game involves something like shooting we can run real time adjustment
    calculations to the shooters accuracy so our driver may shoot from the best angle possible

    The entire point of this script is to make the drivers life easy as possible while still giving
    them ample control of the vehicle. Eliminating a button press to queue something in an indexer and
    replacing it with a sensor or something is a great example of this mentality, the driver does not lose
    control of the robot or any of its behaviors that they would want controlled.

    An example of the automation mindset being taken too far is if we were to program paths into the buttons, if there was no way for the
    driver to control their path this could cause a lot of issues in the main game. There are too many variables with 3 other robots
    potentially playing defense
     */

    @Override
    protected void onInitialize() {
        try {
            Scanner fileParser = new Scanner(new File(Constants.SAVE_FILE_PATH));
            localizer = new Localizer(hardwareMap, new Point(fileParser.nextDouble(), fileParser.nextDouble()), fileParser.nextDouble());
            driveTrain = new MecanumDriveTrain(hardwareMap);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        shooter = new Shooter(hardwareMap);
        driveThread = new Thread(){
            public void run(){
                while(opModeIsActive()){
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
        armThread = new Thread(){
            public void run(){
                while(opModeIsActive()){
                    //do a variety of conditional things to manipulate scoring
                    if(gamepad1.right_trigger > 0.3){
                        shooter.intake(INTAKE_MOTOR_POWER);
                     }

                }
            }
        };
    }

    @Override
    protected void onStart() {
        armThread.start();
        driveThread.start();
        while(opModeIsActive()); //this is here to stall the main thread so the OpMode does not prematurely end

    }

    @Override
    protected void onStop() {

    }
}
