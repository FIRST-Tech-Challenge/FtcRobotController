package org.firstinspires.ftc.teamcode.opmodes.mechBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.hardware.MechBot.ToboMech;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
@Disabled
@TeleOp(name = "TeleDrive", group = "MechBot")
public class TeleDrive_LinearOpMode extends LinearOpMode {
    private DatagramSocket socket;
    private boolean canRunGamepadThread;
    private Thread gamepadHandler;

    private void startGamepadHandlerThread() {
        telemetry.setAutoClear(true);
        gamepadHandler = new Thread(new Runnable() {
            @Override
            public void run() {
                while (canRunGamepadThread) {
                    String gamepadAction = "";
                    try {
                        byte[] buffer = new byte[1024];
                        DatagramPacket response = new DatagramPacket(buffer, buffer.length);
                        socket.receive(response);
                        gamepadAction = new String(buffer, 0, response.getLength());
                    } catch (Exception ignore) {

                    }

                    if (!gamepadAction.isEmpty()) {
                        if (gamepadAction.contains("G1")) {
                            if (gamepadAction.contains("_A")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.a = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.a = false;
                                }
                            }
                            if (gamepadAction.contains("_B")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.b = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.b = false;
                                }
                            }
                            if (gamepadAction.contains("_X")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.x = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.x = false;
                                }
                            }
                            if (gamepadAction.contains("_Y")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.y = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.y = false;
                                }
                            }
                            if (gamepadAction.contains("_D")) {
                                if (gamepadAction.contains("UP")) {
                                    gamepad1.dpad_up = true;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_left = false;
                                    gamepad1.dpad_right = false;
                                }
                                if (gamepadAction.contains("DOWN")) {
                                    gamepad1.dpad_down = true;
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_left = false;
                                    gamepad1.dpad_right = false;
                                }
                                if (gamepadAction.contains("LEFT")) {
                                    gamepad1.dpad_left = true;
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_right = false;
                                }
                                if (gamepadAction.contains("RIGHT")) {
                                    gamepad1.dpad_right = true;
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_left = false;
                                }
                                if (gamepadAction.contains("NONE")) {
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_left = false;
                                    gamepad1.dpad_right = false;
                                }
                            }
                            if (gamepadAction.contains("_RT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.right_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.right_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_LT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.left_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.left_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_RB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.right_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.right_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_LB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.left_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.left_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_RS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.right_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.right_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_LS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.left_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.left_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_START")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.start = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.start = false;
                                }
                            }
                            if (gamepadAction.contains("_BACK")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.back = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.back = false;
                                }
                            }
                            if (gamepadAction.contains("_Jx")) {
                                gamepad1.left_stick_x = Float.parseFloat(gamepadAction.replace("G1_Jx_", ""));
                            }
                            if (gamepadAction.contains("_Jy")) {
                                gamepad1.left_stick_y = Float.parseFloat(gamepadAction.replace("G1_Jy_", ""));
                            }
                            if (gamepadAction.contains("_Jz")) {
                                gamepad1.right_stick_x = Float.parseFloat(gamepadAction.replace("G1_Jz_", ""));
                            }
                            if (gamepadAction.contains("_Jrz")) {
                                gamepad1.right_stick_y = Float.parseFloat(gamepadAction.replace("G1_Jrz_", ""));
                            }
                        }
                        if (gamepadAction.contains("G2")) {
                            if (gamepadAction.contains("_A")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.a = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.a = false;
                                }
                            }
                            if (gamepadAction.contains("_B")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.b = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.b = false;
                                }
                            }
                            if (gamepadAction.contains("_X")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.x = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.x = false;
                                }
                            }
                            if (gamepadAction.contains("_Y")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.y = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.y = false;
                                }
                            }
                            if (gamepadAction.contains("_D")) {
                                if (gamepadAction.contains("UP")) {
                                    gamepad2.dpad_up = true;
                                    gamepad2.dpad_down = false;
                                    gamepad2.dpad_left = false;
                                    gamepad2.dpad_right = false;
                                }
                                if (gamepadAction.contains("DOWN")) {
                                    gamepad2.dpad_down = true;
                                    gamepad2.dpad_up = false;
                                    gamepad2.dpad_left = false;
                                    gamepad2.dpad_right = false;
                                }
                                if (gamepadAction.contains("LEFT")) {
                                    gamepad2.dpad_left = true;
                                    gamepad2.dpad_up = false;
                                    gamepad2.dpad_down = false;
                                    gamepad2.dpad_right = false;
                                }
                                if (gamepadAction.contains("RIGHT")) {
                                    gamepad2.dpad_right = true;
                                    gamepad2.dpad_up = false;
                                    gamepad2.dpad_down = false;
                                    gamepad2.dpad_left = false;
                                }
                                if (gamepadAction.contains("NONE")) {
                                    gamepad2.dpad_up = false;
                                    gamepad2.dpad_down = false;
                                    gamepad2.dpad_left = false;
                                    gamepad2.dpad_right = false;
                                }
                            }
                            if (gamepadAction.contains("_RT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.right_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.right_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_LT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.left_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.left_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_RB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.right_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.right_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_LB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.left_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.left_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_RS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.right_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.right_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_LS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.left_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.left_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_START")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.start = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.start = false;
                                }
                            }
                            if (gamepadAction.contains("_BACK")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.back = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.back = false;
                                }
                            }
                            if (gamepadAction.contains("_Jx")) {
                                gamepad2.left_stick_x = Float.parseFloat(gamepadAction.replace("G2_Jx_", ""));
                            }
                            if (gamepadAction.contains("_Jy")) {
                                gamepad2.left_stick_y = Float.parseFloat(gamepadAction.replace("G2_Jy_", ""));
                            }
                            if (gamepadAction.contains("_Jz")) {
                                gamepad2.right_stick_x = Float.parseFloat(gamepadAction.replace("G2_Jz_", ""));
                            }
                            if (gamepadAction.contains("_Jrz")) {
                                gamepad2.right_stick_y = Float.parseFloat(gamepadAction.replace("G2_Jrz_", ""));
                            }
                        }
                    }
                }
                gamepadHandler.interrupt();
            }
        });
        gamepadHandler.setName("Gamepad Handler Thread");
        gamepadHandler.setPriority(Thread.NORM_PRIORITY);
        gamepadHandler.start();
    }

    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    private EventManager eventManager1;
    private EventManager eventManager2;

    @Override
    public void runOpMode() throws InterruptedException {

        String address = "192.168.43.1"; //Check "Program and Manage" tab on the Driver Station and verify the IP address
        int port = 11039; //Change as needed
        canRunGamepadThread = false;

        try {
            this.socket = new DatagramSocket(port);
        } catch (Exception ex) {
            telemetry.addData("Exception: ", ex.getMessage());
        }

        log.info("RoboMech TeleOp runOpMode() starts (CPU_time = %.2f sec)", getRuntime());
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboMech robot = new ToboMech();
        robot.configureLogging(robot.getName(),LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);

        log.info("RoboMech TeleOp finished configuration (CPU_time = %.2f sec)", getRuntime());

        try {
            // configure robot and reset all hardware
            robot.configure(configuration, telemetry, Robot2.ProgramType.TELE_OP);
            configuration.apply();
            robot.isTeleOpAfterAuto = false;
            robot.initSetup(Robot2.ProgramType.TELE_OP, ToboMech.StartPosition.OUT, configuration); // check
            robot.reset(false);

            eventManager1 = new EventManager(gamepad1, true);
            eventManager2 = new EventManager(gamepad2, true);

            robot.mainTeleOp(eventManager1);

            robot.showStatus(getRuntime());

        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }
        log.info("RoboRuck TeleOp finished initialization (CPU_time = %.2f sec)", getRuntime());
        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        canRunGamepadThread = true;

        startGamepadHandlerThread();

        //CUSTOM CODE GOES HERE
        robot.initAfterStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                try {
                    eventManager1.processEvents();
                    eventManager2.processEvents();
                    TaskManager.processTasks();
                } catch (Exception E) {
                    telemetry.addData("Error in event handler", E.getMessage());
                    handleException(E);
                    Thread.sleep(5000);
                }
            }
            canRunGamepadThread = false;
            socket.close();
        }

        if (robot.chassis!=null && robot.chassis.getGPS()!=null)
            robot.chassis.getGPS().stop();
    }

    protected void handleException(Throwable T) {
        log.error(T.getMessage(), T);
        int linesToShow = 5;
        for(StackTraceElement line : T.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }
}
