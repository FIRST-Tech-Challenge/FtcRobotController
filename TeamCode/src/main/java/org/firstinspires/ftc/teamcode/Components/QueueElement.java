package org.firstinspires.ftc.teamcode.Components;

public class QueueElement {
    public int queueNumber = 0;
    public int after = 0;
    public QueueElement(int queueNum) {
        queueNumber = queueNum;
    }
    public QueueElement(int queueNum, int afterNum) {
        queueNumber = queueNum;
        after = afterNum;
    }
    public boolean ready(){
        return true;
    }
}
