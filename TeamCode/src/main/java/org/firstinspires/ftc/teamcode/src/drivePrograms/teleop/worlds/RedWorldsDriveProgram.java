package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.worlds;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.TripWireDistanceSensor;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@TeleOp(name = "🟥Red Worlds Drive Program🟥")
public class RedWorldsDriveProgram extends TeleOpTemplate {
    protected static BlinkinPattern defaultColor = BlinkinPattern.RED;
    BlinkinPattern currentColor = defaultColor;
    //TripWireDistanceSensor distanceSensor;
    private boolean x_depressed = true;
    private boolean tapeMeasureCtrl = false;

    public void opModeMain() throws InterruptedException {
        this.initAll();
        /*
        distanceSensor = new TripWireDistanceSensor(hardwareMap, "distance_sensor", 8, () -> {
            this.leds.setPattern(BlinkinPattern.BLACK);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ignored) {
            }
            this.leds.setPattern(currentColor);
            return null;

        }, this::opModeIsActive, this::isStopRequested);

         */

        //distanceSensor.start();
        leds.setPattern(defaultColor);

        slide.autoMode();

        telemetry.addData("Initialization", "finished");
        telemetry.update();

        System.gc();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            //Declan's controls
            {
                driveTrain.gamepadControl(gamepad1, gamepad2);
                //Carousel Spinner
                spinner.gamepadControl(gamepad1, gamepad2);
            }


            //Eli's controls
            {
                if (!gamepad2.x){
                    x_depressed = true;
                }
                if (gamepad2.x && x_depressed){
                    tapeMeasureCtrl = !tapeMeasureCtrl;
                    turret.halt();
                }

                if (tapeMeasureCtrl){
                    turret.gamepadControl(gamepad1,gamepad2);
                }else {

                    //Handles Linear Slide Control
                    slide.gamepadControl(gamepad1, gamepad2);

                    //Intake Controls
                    outtake.gamepadControl(gamepad1, gamepad2);

                    intake.gamepadControl(gamepad1,gamepad2);

                    RobotLog.d("I exist and am running");
                    


                    /*
                    FreightFrenzyGameObject currentObj = (FreightFrenzyGameObject) outtake.gamepadControl(gamepad1, gamepad2);
                    RevBlinkinLedDriver.BlinkinPattern colorToChangeTo = defaultColor;
                    if (currentObj != null) {
                        colorToChangeTo = FreightFrenzyGameObject.getLEDColorFromItem(currentObj);
                    }

                    if (colorToChangeTo != currentColor && colorToChangeTo != null) {
                        leds.setPattern(colorToChangeTo);
                        currentColor = colorToChangeTo;
                    }

                     */
                    idle();
                }

            }

        }
    }
}


