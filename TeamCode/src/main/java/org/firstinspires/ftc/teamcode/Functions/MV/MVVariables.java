package org.firstinspires.ftc.teamcode.Functions.MV;


import com.qualcomm.robotcore.hardware.DcMotor;

public class MVVariables {
    // In clasa asta tinem toate variabilele complexe cum ar fi Vector2
    // Aici scriem clase cum am scrie structurile in C

    public static class Vector2{
        // Clasa asta retine un Vector 2, dar poate face mai mult de atat.
        public double x=0;
        public double y=0;

        // poti sa folosesti variabilele astea sau functiile
        // diferenta dintre ele este ca functiile calculeaza de fiecare data valoarea
        // ceea ce poate avea un efect negativ asupra performantei dar in schimb functiile returneaza numarul corect de fiecare data
        // in comparatie cu variabilele care din neatentie pot fi schimbate

        // SA NU MODIFICI VARIABILA ASTA
        // DOAR SA O CITESTI
        public double magnitude;
        public double verticalAngle;
        public double horizontalAngle;

        void Init(){
            magnitude = Magnitude();
            verticalAngle = VerticalAngle();
            horizontalAngle = HorizontalAngle();
        }

        /**
         * This method resets both x,y variables to 0.
         */
        public void Reset(){
            x=0;
            y=0;
        }

        public Vector2(double _x, double _y){
            x=_x;
            y=_y;
            Init();
        }

        /**
         * This method calculates x,y with an origin given (if the vector doesn't have origin 0,0).
         */
        public Vector2(double x1, double x2, double y1, double y2){
            x=x2-x1;
            y=y2-y1;
            Init();
        }

        /**
         * This method calculates module of the vector.
         * @return : module of vector
         */
        public double Magnitude(){
            return Math.sqrt(Math.abs(x*x+y*y));
        }

        /**
         * This method calculates the angle of the vector towards Oy axis (vertical axis).
         * @return : angle towards Oy axis
         */
        public double VerticalAngle(){
            return Math.atan2(x, y);
        }

        // calculam unghiul vectorului fata de axa Ox (axa orizontala)

        /**
         * This method calculates the angle of the vector towards Ox axis (horizontal axis).
         * @return : angle towards Ox axis
         */
        public double HorizontalAngle(){
            return Math.atan2(y, x);
        }

        /**
         * This method inverts the vector.
         */
        public void Invert(){
            double aux;
            aux=x;
            x=y;
            y=aux;
        }

        /**
         * This method adds a new vector.
         * @param vector : (Vector3) given vector.
         */
        public void AddVector(Vector2 vector){
            x += vector.x;
            y += vector.y;
        }

        /**
         * This method decreases the vector variables with another vectors variables.
         * @param vector : (Vector3) given vector
         */
        public void RemoveVector(Vector2 vector){
            x -= vector.x;
            y -= vector.y;
        }

        /**
         * This method multiplies the variables with a given number.
         * @param number : (double) given number
         */
        public void MultiplyNumber(double number){
            x *= number;
            y *= number;
        }
        /**
         * These methods set in particular to each one of the variables x,y negative values.
         */
        public void NegativeX(){
            x=-x;
        }
        public void NegativeY(){
            y=-y;
        }

        /**
         * This method sets all the values of x,y to negative ones.
         */
        public void Negative(){
            x=-x;
            y=-y;
        }
        /**
         * This method transforms the vector on another plan, which has the origin with the current plan but it's at a different angle.
         * This method will try not to modify the direction of the vector.
         * @param angle : -+90 will return an error
         * @return : true if the old magnitude matches with the new
         */
        public boolean Rotate(double angle){
            double magnitudeDelta=magnitude;
            double radians = Math.toRadians(angle);
            double xAlpha = Math.cos(radians)*x-Math.sin(radians)*y;
            double yAlpha = Math.sin(radians)*x+Math.cos(radians)*y;
            x = xAlpha;
            y = yAlpha;
            return magnitudeDelta==Magnitude();
            //x2=cosβx1−sinβy1
            //y2=sinβx1+cosβy1
        }
        /**
         * This method transforms the vector on another plan, which has the origin with the current plan but it's at a different angle.
         * This method will try not to modify the direction of the vector.
         * @param radians : -+pi/2 will return an error
         * @return : true if the old magnitude matches with the new
         */
        public boolean RotateRadians(double radians){
            double magnitudeDelta=magnitude;
            double xAlpha = Math.cos(radians)*x-Math.sin(radians)*y;
            double yAlpha = Math.sin(radians)*x+Math.cos(radians)*y;
            x = xAlpha;
            y = yAlpha;
            return magnitudeDelta==Magnitude();
            //x2=cosβx1−sinβy1
            //y2=sinβx1+cosβy1
        }

        /**
         * This method transforms the vector on another plan, which has the origin with the current plan but it's at a different angle.
         * This method will try not to modify the direction of the vector.
         * @param angle :
         * @return : true if there's an error
         */
        public boolean TranformToNewPlane(double angle){
            angle=NormalizeAngle(angle);
            double newAngle = Math.toRadians(angle);
            double y1 = Math.sin(newAngle)* magnitude;
            double x1 = Math.cos(newAngle)* magnitude;
            if(new Vector2(x1, y1).Magnitude()== magnitude)
            {
                x=x1;
                y=y1;
                return false;
            }
            else {
                x=x1;
                y=y1;
                return true;
            }

        }

        /**
         * This method helps the TransformToNewPlane method, which ensures that the angle doesn't get past 360 degrees or below 0 degrees.
         * @param angle : (double) given angle
         * @return : normalized angle
         */
        private double NormalizeAngle(double angle){
            double newAngle=angle;
            while(newAngle<=0){
                newAngle=newAngle+360;
            }
            while(newAngle>360){
                newAngle=newAngle-360;
            }
            return angle;
        }

        /**
         * @return : This returns x,y and module of vector.
         */
        public String SimpleData(){
            return "x: "+ x + " y: " + y + " m: " + Magnitude();
        }

        /**
         * @return : This returns all the data that we have.
         */
        public String ReturnData(){
            Init();
            return SimpleData() + " vAngle: " + VerticalAngle() + " hAngle: " + HorizontalAngle();
        }


    }


    public static class MotorHolder{
        DcMotor leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack;

        /**
         * USAGE to keep all the moving motors, write this in the constructor of a class:
         * public CLASSNAME(Variables.MotorHolder _MotorHolder){
         * leftMotor = _MotorHolder.getLeftMotor();
         * rightMotor = _MotorHolder.getRightMotor();
         * leftMotorBack = _MotorHolder.getLeftMotorBack();
         * rightMotorBack = _MotorHolder.getRightMotorBack();
         */

        /**
         * This method initialises the motors.
         * @param _LMF : left motor front
         * @param _RMF : right motor front
         * @param _LMB : left motor back
         * @param _RMB : right motor back
         */
        public MotorHolder(DcMotor _LMF, DcMotor _RMF, DcMotor _LMB, DcMotor _RMB){
            leftMotorFront = _LMF;
            rightMotorFront = _RMF;
            leftMotorBack = _LMB;
            rightMotorBack = _RMB;

        }

        /**
         * These methods return specific motors.
         */
        public DcMotor getLeftMotorFront(){
            return leftMotorFront;
        }
        public DcMotor getRightMotorFront(){
            return rightMotorFront;
        }
        public DcMotor getLeftMotorBack(){
            return leftMotorBack;
        }
        public DcMotor getRightMotorBack(){
            return rightMotorBack;
        }
        public double getLeftMotorPower(){
            return leftMotorFront.getPower();
        }
        public double getRightMotorPower(){
            return rightMotorFront.getPower();
        }
        public double getLeftMotorBackPower(){
            return leftMotorBack.getPower();
        }
        public double getRightMotorBackPower(){
            return rightMotorBack.getPower();
        }

        /**
         * This method returns all the motors with text.
         */
        public String getMotorPower(){
            return "leftMotor: " + getLeftMotorPower()+" rightMotor: "+getRightMotorPower()+" leftMotorBack: "
                    +getLeftMotorBackPower()+" rightMotorBack: "+getRightMotorBackPower();
        }


    }

    public static class Vector3{
        public double x=0;
        public double y=0;
        public double z=0;

        /**
         *This class stores a Vector3.
         * You cand either use the variables or the methods. The difference is that:
         * On one hand methods always calculate the value, which can have a negative effect when it comes to performance
         * but on the other hand methods always calculate the correct number, unlike the variables which by negligence can be changed.
         */


        public Vector3(double _x, double _y, double _z){
            x=_x;
            y=_y;
            z=_z;
        }

        /**
         * This method calculates x,y,z with an origin given (if the vector doesn't have origin 0,0).
         */
        public Vector3(double x1, double x2, double y1, double y2, double z1, double z2){
            x=x2-x1;
            y=y2-y1;
            z=z2-z1;
        }

        /**
         * This method adds a new vector.
         * @param vector : (Vector3) given vector.
         */
        public void AddVector(Vector3 vector){
            x += vector.x;
            y += vector.y;
            z += vector.z;
        }

        /**
         * This method decreases the vector variables with another vectors variables.
         * @param vector : (Vector3) given vector
         */
        public void RemoveVector(Vector3 vector){
            x -= vector.x;
            y -= vector.y;
            z -= vector.z;
        }

        /**
         * This method multiplies the variables with a given number.
         * @param number : (double) given number
         */
        public void MultiplyNumber(double number){
            x *= number;
            y *= number;
            z *= number;
        }

        /**
         * This method sets all the variables to a positive value.
         */
        public void Positive(){
            x=Math.abs(x);
            y=Math.abs(y);
            z=Math.abs(z);
        }

        /**
         * @return : (boolean) This returns true - if all variables are negative OR false - contrary.
         */
        public boolean IsFullNegative(){
            return x<0&&y<0&&z<0;
        }

        /**
         * This method resets only negative values to 0.
         */
        public void ResetNegativeValues(){
            if(x<0)x=0;
            if(y<0)y=0;
            if(z<0)z=0;
        }

        /**
         * These methods set in particular to each one of the variables x,y,z negative values.
         */
        public void NegativeX(){
            x=-x;
        }
        public void NegativeY(){
            y=-y;
        }
        public void NegativeZ(){
            z=-z;
        }

        /**
         * This method sets all the values of x,y,z to negative ones.
         */
        public void Negative(){
            x=-x;
            y=-y;
            z=-z;
        }

        /**
         * This method resets x,y,z to 0.
         */
        public void Reset(){
            x=0;
            y=0;
            z=0;
        }

        /**
         * @return : This returns x,y and module of vector.
         */
        public String SimpleData(){
            return "x: "+ x + " y: " + y + " z: " + z;
        }

        /**
         * @return : This returns all the data that we already have.
         */
        public String ReturnData(){
            return SimpleData();
        }


    }




}
