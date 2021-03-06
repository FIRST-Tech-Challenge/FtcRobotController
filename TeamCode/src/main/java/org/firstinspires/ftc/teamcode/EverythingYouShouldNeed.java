//TELEOP
//Wheels
/*
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
            double lfPower;
            double rfPower;
            double lbPower;
            double rbPower;
            double SoNPower;

            lfPower = 0.0f ;
            rfPower = 0.0f ;
            lbPower = 0.0f ;
            rbPower = 0.0f ;

            if (abs(gamepad1.left_stick_y) < 0.2 && abs(gamepad1.left_stick_x) > 0.2){
                lfPower = -gamepad1.left_stick_x;
                lbPower = gamepad1.left_stick_x;
            } else
            if (abs(gamepad1.left_stick_y) < 0.2 && abs(gamepad1.left_stick_x) < 0.2){
                lfPower = 0.15 * ((gamepad1.left_stick_y - gamepad1.left_stick_x)/(abs(gamepad1.left_stick_y - gamepad1.left_stick_x)));
                lbPower = 0.15 * ((gamepad1.left_stick_y + gamepad1.left_stick_x)/(abs(gamepad1.left_stick_y + gamepad1.left_stick_x)));
            } else
            if (abs(gamepad1.left_stick_y) > 0.2 && abs(gamepad1.left_stick_x) > 0.2){
                lfPower = gamepad1.left_stick_y - gamepad1.left_stick_x ;
                lbPower = gamepad1.left_stick_y + gamepad1.left_stick_x ;
            } else
            if (abs(gamepad1.left_stick_y) > 0.2 && abs(gamepad1.left_stick_x) < 0.2){
                lfPower = gamepad1.left_stick_y;
                lbPower = gamepad1.left_stick_y;
            }

            if (abs(gamepad1.left_stick_y) < 0.05 && abs(gamepad1.left_stick_x) < 0.05){
                lfPower = 0.0f ;
                lbPower = 0.0f ;
            }

            if (abs(gamepad1.right_stick_y) < 0.2 && abs(gamepad1.right_stick_x) > 0.2){
                rfPower = gamepad1.right_stick_x;
                rbPower = -gamepad1.right_stick_x;
            } else
            if (abs(gamepad1.right_stick_y) < 0.2 && abs(gamepad1.right_stick_x) < 0.2){
                rfPower = 0.15 * ((gamepad1.right_stick_y + gamepad1.right_stick_x)/(abs(gamepad1.right_stick_y + gamepad1.right_stick_x)));
                rbPower = 0.15 * ((gamepad1.right_stick_y - gamepad1.right_stick_x)/(abs(gamepad1.right_stick_y - gamepad1.right_stick_x)));
            } else
            if (abs(gamepad1.right_stick_y) > 0.2 && abs(gamepad1.right_stick_x) > 0.2){
                rfPower = gamepad1.left_stick_y + gamepad1.right_stick_x ;
                rbPower = gamepad1.left_stick_y - gamepad1.right_stick_x ;
            } else
            if (abs(gamepad1.right_stick_y) > 0.2 && abs(gamepad1.right_stick_x) < 0.2){
                rfPower = gamepad1.right_stick_y;
                rbPower =  gamepad1.right_stick_y;
            }
            if (abs(gamepad1.right_stick_y) < 0.05 && abs(gamepad1.right_stick_x) < 0.05){
                rfPower = 0.0f ;
                rbPower = 0.0f ;
            }
            if (gamepad1.right_bumper){
                lf.setPower(lfPower *0.5);
                rf.setPower(rfPower *0.5);
                lb.setPower(lbPower *0.5);
                rb.setPower(rbPower *0.5);
            } else{
                lf.setPower(lfPower *0.25);
                rf.setPower(rfPower *0.25);
                lb.setPower(lbPower *0.25);
                rb.setPower(rbPower *0.25);
            }*/
//SoN
/*
    private CRServo SoN = null;
        SoN = hardwareMap.get(CRServo.class, "SoN");
        SoN.setDirection(CRServo.Direction.FORWARD);
        double cPower = 0;
            if(gamepad2.a){
                cPower = 1;
            }
            else if(gamepad2.b){
                cPower = -1;
            }
            else {
                cPower = 0;
            }
            SoN.setPower(cPower);
 */
//Shooter
/*
    private DcMotor spindoctor1;
    private DcMotor spindoctor2;
        spindoctor1 = hardwareMap.get(DcMotorEx.class, "spindoctor1");
        spindoctor2 = hardwareMap.get(DcMotorEx.class, "spindoctor2");
        spindoctor1.setDirection(DcMotor.Direction.FORWARD);
        spindoctor2.setDirection(DcMotor.Direction.REVERSE);
whatever you want the shooter to be bound to, set it eual to something like
double spindoctor1Power
spindoctor1Power = 'button or joystick,
spindoctor1.setPower(spindoctor1Power)
 */