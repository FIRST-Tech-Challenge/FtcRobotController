package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import static java.lang.Math.abs;

/**
 * The purpose of this class is to take the current voltage of the battery and calculate
 * how fast will the robot go at the current power level
 * USAGE:
 * long waitTime = voltageReader.GetWaitTime(100, 1);
 * for(int i=0; i<waitTime;i++){
 *     move.MoveFull(2);
 *     telemetry.addLine(positionCalculator.NewUpdate(getRuntime()));
 *     sleep(1);
 * }
 * OLD USAGE:
 * move.MoveFull(1);
 * sleep(voltageReader.GetWaitTime(100f, 1));
 */

public class VoltageReader {
    double currentVoltageOffset =0.3f;
    double currentVoltage = 0;
    VoltageSensor voltageSensor;

    /**
     * Class must be initialised with a reference at voltageSensor, more specifically:
     * VoltageSensor VS = this.hardwareMap.voltageSensor.iterator().next();
     *
     */

    public VoltageReader(VoltageSensor VS){
        currentVoltage = VS.getVoltage()+ currentVoltageOffset;
        voltageSensor = VS;
        InitStruct();
        ProcessData();
    }

    /**
     * This method reads (getter) for currentVoltage (variable).
     * @return : (double) This returns current voltage.
     */
    public double ReturnVoltage(){
        currentVoltage = voltageSensor.getVoltage()+ currentVoltageOffset;
        return currentVoltage;
    }

    /**
     * We put all the calculations in a struct, then we process those calculations, and after that we decide
     * how far will the robot go in the given situation.
     */

    /**
     * Struct must be completed here (in code) by hand.
     * distance = cm (not meters)
     * time = seconds
     * voltage = V (volt)
     */

    class data {
        float distance[]= new float[100];
        float time[]= new float[100];
        float voltage[]= new float[100];
        int direction[]= new int[100];
        // 1 front - back
        // 2 slide left - right
        // 3 rotation left - right
    }
    data currentData = new data();

    int counter=0;

    /**
     * This method helps at the initialisation of the struct.
     * @param distance : self explanatory
     * @param time : self explanatory
     * @param voltage : self explanatory
     * @param direction : self explanatory
     */
    void AddData(float distance, float time, float voltage, int direction){
        currentData.distance[counter]=distance;
        currentData.time[counter]=time;
        currentData.voltage[counter]=voltage;
        currentData.direction[counter]=direction;
        counter++;
    }
    
    /**
     * This method reads a specific value.
     * @param number : 1 - distance; 2 - time; 3 - voltage; 4 - direction
     * @param counter :
     * @return : (float) distance/time/voltage/direction
     */
    float ReadData(int number, int counter)
    {
        switch(number){
            case 1:
                return currentData.distance[counter];
            case 2:
                return currentData.time[counter];
            case 3:
                return currentData.voltage[counter];
            case 4:
                return currentData.direction[counter];
        }
        return 1;
    }

    /**
     * This method initialises the struct with current values that we have.
     */
    void InitStruct(){
        //AddData(300, 5, 11.98f, 1);
        //AddData(300, 6.25f, 11.00f, 1);
        //AddData(300, 5.90f, 11.00f, 1);
        //AddData(300, 6.70f, 11.00f, 1);

        //AddData(360, 4.77f, 11.00f, 3);
        //AddData(360, 4.36f, 11.00f, 3);
        //AddData(360, 4.50f, 10.79f, 3);
        //AddData(360, 4.99f, 10.72f, 3);

        //AddData(360, 3.55f, 12.92f, 3);
        //AddData(360, 3.21f, 12.93f, 3);
        //AddData(360, 3.30f, 12.92f, 3);
        //AddData(360, 3.49f, 12.92f, 3);
        /**
         * De aici am modifcat
         */
        AddData(300, 5.00f, 13.16f, 1);
//        AddData(300, 5.25f, 13.11f, 1);
//        AddData(300, 4.94f, 13.00f, 1);
//        AddData(300, 4.93f, 12.99f, 1);

//         AddData(300,6.47f,10.95f,1);
        AddData(300,3.10f,14.27f,1);
        AddData(300,3.66f,14.18f,1);
//        AddData(300,3.5f,13.80f,1);
        AddData(300,3.25f,13.40f,1);
        AddData(300,3.41f,13.37f,1);
//        AddData(260,1.92f,12.54f,1);
//        AddData(300,2.84f,12.54f,1);
//
//        AddData(300, 2.86f, 13.79f, 1);
        AddData(300, 2.86f, 13.68f, 1);
//        AddData(300, 2.58f, 13.63f, 1);
//        AddData(300, 2.56f, 13.60f, 1);
//        AddData(300, 2.87f, 13.56f, 1);
//        AddData(300, 2.9f, 13.52f, 1);





        AddData(300,4.0f,13.94f,2);
//        AddData(300,4.33f,13.84f,2);
//        AddData(300, 6.75f, 12.42f, 2);
//        AddData(300, 6.92f, 12.42f, 2);
//        AddData(300, 6.76f, 12.40f, 2);
//        AddData(300, 6.96f, 12.38f, 2);

        //new data
        AddData(300, 8.6f, 12.68f, 2);
//        AddData(300, 08.5f, 12.58f, 2);
        AddData(300, 8.62f, 12.65f, 2);
//        AddData(300, 8.44f, 12.56f, 2);
//        AddData(300, 7.11f, 13.76f, 2);
        AddData(300, 7.76f, 13.63f, 2);
//        AddData(300, 7.54f, 13.59f, 2);
        AddData(300, 7.94f, 13.4f, 2);




        //AddData(300, 6.96f, 12.38f, 2);
        //AddData(300, 7.18f, 12.34f, 2);

        //AddData(360, 3.14f, 13.75f, 3);
        //AddData(360, 3.29f, 13.75f, 3);
        //AddData(360, 3.21f, 13.75f, 3);
        //AddData(360, 3.38f, 13.72f, 3);

        //AddData(360, 3.14f, 13.72f, 3);

        //AddData(360, 3.92f, 12.14f, 3);
        //AddData(360, 4.01f, 12.14f, 3);
        //AddData(360, 3.61f, 12.14f, 3);
        //AddData(360, 4.22f, 12.14f, 3);

        AddData(360, 4f, 11.72f, 3);
//        AddData(360, 3.84f, 11.72f, 3);
//        AddData(360, 3.80f, 11.72f, 3);
//        AddData(360, 0.70f, 12.56f, 3);
//        AddData(360, 1.02f, 12.55f, 3);
        AddData(360, 0.95f, 12.55f, 3);

//        AddData(360, 3.85f, 11.72f, 3);

//        AddData(360,1.50f,12.75f,3);
//        AddData(360,1.08f,12.73f,3);
        AddData(360,1.51f,13.81f,3);
//        AddData(360,1.21f,13.77f,3);
//        AddData(360,1.68f,13.85f,3);
//        AddData(360,1.46f,13.31f,3);
//        AddData(360,1.0f,14.18f,3);
        AddData(360,0.86f,14.10f,3);
        //AddData(300, 4f, 14f, 2);
        //AddData(300, 4.20f, 13.99f, 2);
        //AddData(300, 4.12f, 13.98f, 2);

        //AddData(300, 7.83f, 10.20f, 2);
        //AddData(300, 7.80f, 10.12f, 2);
        //AddData(300, 8.12, 10f, 2);

    }

    /**
     * 1 = back - front
     * 2 = slide right - left
     * 3 = rotation right - left
     * These variables store the quotient(catul impartirii) from time and voltage
     */

    float currentTime1 =0;
    float currentVoltage1 =0;
    float currentTime2 =0;
    float currentVoltage2 =0;
    float currentTime3 =0;
    float currentVoltage3 =0;

    /**
     * This method reads the above variables.
     * @param number : number of case (variable that is wanted)
     * @return : the wanted variable
     */
    public float CurrentTimeRead(int number){
        switch(number){
            case 1:
                return currentTime1;
            case 2:
                return currentVoltage1;
            case 3:
                return currentTime2;
            case 4:
                return currentVoltage2;
            case 5:
                return currentTime3;
            case 6:
                return currentVoltage3;
        }
        return 1;
    }


    /**
     * This method processes the actual data, adds up all the values and calculates the average
     */
    void ProcessData(){
        // for 1
        float timeSum=0;
        float voltageSum=0;
        int aux=1;
        for(int index=0;index<=counter;index++){
            if(1==ReadData(4, index)){
                timeSum=timeSum+(ReadData(2, index));
                voltageSum=voltageSum+ReadData(3, index);
                aux++;
            }
        }
        if(aux!=1){
            aux=aux-1;
            currentTime1 =timeSum/(aux);
            currentVoltage1 =voltageSum/(aux);
        }

        // for 2
        timeSum=0;
        voltageSum=0;
        aux=1;
        for(int i=0;i<=counter;i++){
            if(2==ReadData(4, i)){
                timeSum=timeSum+(ReadData(2, i));
                voltageSum=voltageSum+ReadData(3, i);
                aux++;
            }
        }
        if(aux>=1){
            aux=aux-1;
            currentTime2 =timeSum/(aux);
            currentVoltage2 =voltageSum/(aux);
        }

        // for 3
        timeSum=0;
        voltageSum=0;
        aux=1;
        for(int i=0;i<=counter;i++){
            if(3==ReadData(4, i)){
                timeSum=timeSum+(ReadData(2, i));
                voltageSum=voltageSum+ReadData(3, i);
                aux++;
            }
        }
        if(aux!=1){
            aux=aux-1;
            currentTime3 =timeSum/(aux);
            currentVoltage3 =voltageSum/(aux);

            //CurrentTime3=Math.round(Math.pow(timeSum, 1/aux));
            //CurrentVoltage3=Math.round(Math.pow(voltageSum, 1/aux));

        }

    }

    /**
     * This method calculates how much time does the robot need to wait to traverse a specific distance because when the battery is lower he'll go slower.
     * @param distance : given distance
     * @param direction : given direction : 1 = front - back; 2 = slide right - left; 3 = rotation right - left
     * @return
     */
    public long GetWaitTime(float distance, int direction){
        double auxSeconds=0;
        double finalSeconds=0;
        switch (direction){
            case 1:
                auxSeconds = currentTime1 * currentVoltage1 /ReturnVoltage();
                /**
                 * Above, we find how many seconds per 300 cm for the current voltage.
                 *
                 * Below, we find the time for our distance.
                 */
                finalSeconds=abs((distance*auxSeconds)/300*1000-550*distance/150);
                return (long)finalSeconds;
            case 2:
                auxSeconds = currentTime2 * currentVoltage2 /ReturnVoltage();
                /**
                 * Above, we find how many seconds per 300 cm for the current voltage.
                 *
                 * Below, we find the time for our distance.
                 */
                finalSeconds=(distance*auxSeconds)/300*1000-700*distance/150;
                return (long)finalSeconds;
            case 3:
                auxSeconds = currentTime3 * currentVoltage3 /ReturnVoltage();
                /**
                 * Above, we find how many seconds per 300 cm for the current voltage.
                 *
                 * Below, we find the time for our distance.
                 */
                finalSeconds=(distance*auxSeconds)/360*1000-840*distance/360;
                return (long)finalSeconds;
        }
        return 1;
    }

    @Deprecated
    public float GetDistance(float time, int direction)
    {
        double sAux=GetWaitTime(1, direction)/1000;
        double sFinal=(time*sAux)/1;
        return (float)sFinal;
    }



}
