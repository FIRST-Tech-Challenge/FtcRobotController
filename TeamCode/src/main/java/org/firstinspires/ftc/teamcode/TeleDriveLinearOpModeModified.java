package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.net.DatagramPacket;
import java.net.DatagramSocket;

@TeleOp(name = "TeleDriveTeleOp", group = "")
public class TeleDriveLinearOpModeModified extends LinearOpMode {
    private DatagramSocket socket;
    private boolean canRunGamepadThread;
    private Thread gamepadHandler;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor intakeMotor;
    private DcMotor shooterMotor;
    private DcMotor wobbleArmMotor;
    private Servo shooterServo;
    private Servo wobbleArmServo;

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

    @Override
    public void runOpMode() {

        String address = "192.168.43.1";
        int port = 11039;
        canRunGamepadThread = false;

        try {
            this.socket = new DatagramSocket(port);
        } catch (Exception ex) {
            telemetry.addData("Exception: ", ex.getMessage());
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Connect your server to " + address + ":" + port, "");
        telemetry.update();

        waitForStart();

        canRunGamepadThread = true;

        startGamepadHandlerThread();


        //CUSTOM CODE GOES HERE
        frontRightMotor  = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        wobbleArmMotor = hardwareMap.get(DcMotor.class, "wobbleArmMotor");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        wobbleArmServo = hardwareMap.get(Servo.class, "wobbleArmServo");


        // Set drivetrain motors to brake when power is set to 0.
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse left side drive train motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse shooter motor so it goes in the correct direction
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse wobble arm motor
       // wobbleArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set wobble arm motor encoder to 0
        wobbleArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set wobble arm motor to use encoder
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set servo position to 0
        shooterServo.setPosition(0);
        wobbleArmServo.setPosition(0);

    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
          // Setup a variable for each drive wheel to save power level for telemetry
          double y = -gamepad1.left_stick_y; // Remember, this is reversed!
          double x = gamepad1.left_stick_x * 1.5; // Counteract imperfect strafing;
          double rx = gamepad1.right_stick_x;

          frontLeftMotor.setPower(y + x + rx);
          backLeftMotor.setPower(y - x + rx);
          frontRightMotor.setPower(y - x - rx);
          backRightMotor.setPower(y + x - rx);

          if(gamepad1.x) {
              frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
              backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
              frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
              backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
          } else {
              frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
              backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
              frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
              backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
          }

          //Set gamepad2 assignments

          intakeMotor.setPower(gamepad2.right_trigger);
          intakeMotor.setPower(gamepad1.right_trigger);

          if(gamepad2.b) {
              shooterMotor.setPower(-1);
          }
          if(gamepad2.y) {
              shooterMotor.setPower(0);
          }

          if (gamepad2.dpad_right) {
              wobbleArmMotor.setTargetPosition(230);
              wobbleArmMotor.setPower(0.5);
              wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              sleep(500);
              wobbleArmMotor.setPower(0);
          }
          if (gamepad2.dpad_left) {
              wobbleArmMotor.setTargetPosition(50);
              wobbleArmMotor.setPower(0.5);
              wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              sleep(500);
              wobbleArmMotor.setPower(0);
          }

          if(gamepad2.x) {
              shooterServo.setPosition(0.35);
          } else {
              shooterServo.setPosition(0);
          }
          if (gamepad2.dpad_up) {
              wobbleArmServo.setPosition(0.35);
          }
          if (gamepad2.dpad_down) {
              wobbleArmServo.setPosition(0.0);
          }
      }
    }

        canRunGamepadThread = false;
        socket.close();
    }
}
