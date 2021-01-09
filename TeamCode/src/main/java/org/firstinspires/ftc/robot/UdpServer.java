package org.firstinspires.ftc.robot;



import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.concurrent.Semaphore;

public class UdpServer implements Runnable{
    //the port of the client
    private final int clientPort;
    public static boolean kill = false;

    public UdpServer(int clientPort) {
        this.clientPort = clientPort;
    }

    //guards thread collisions
    private Semaphore sendLock = new Semaphore(1);



    //this is the time of the last update in milliseconds
    private long lastSendMillis = 0;

    /**
     * This runs repeatedly (it's own thread). It looks to see if there are any messages to send
     * and if so which to send.
     */
    @Override
    public void run() {
        while(true){
            if(kill){break;}
            try {
                //never send data too fast
                if(System.currentTimeMillis()-lastSendMillis < 50) {
                    continue;
                }
                //set the last send time
                lastSendMillis = System.currentTimeMillis();

                //wait for semaphore to be available
                sendLock.acquire();


                //We will send either the current update or the last update depending on
                //if we are using the currentUpdate String or not
                if(currentUpdate.length() > 0){
                    //send the current update
                    splitAndSend(currentUpdate);
                    //now we scrap everything in currentUpdate to flag it is empty
                    currentUpdate = "";
                }else{
//                    //if we are here, the currentUpdate is empty
//                    if(lastUpdate.length() > 0){
//                        splitAndSend(lastUpdate);
//                        //now we scrap everything in lastUpdate to flag it is empty
//                        lastUpdate = "";
//                    }
                }

                //release the semaphore
                sendLock.release();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }

    /**
     * This method uses sendUdpRAW but will split up the message if it is too long
     * @param message the message you wish to send
     */
    public void splitAndSend(String message) {
//        System.out.println("sending: " + message);
        //these are the ranges we are sending over
        int startIndex = 0;
        int endIndex;

        do {
            //We will start the end index approximately 600 away. But can't be greater
            //than the message length
            endIndex = Range.clip(startIndex + 600, 0, message.length() - 1);


            //Now we will move backwards scanning for the end of this message
            //if this character is a separator, we can mark the end index
            while (message.charAt(endIndex) != '%') {
                endIndex--;//move backwards searching for the separator
            }

            //need to add one to the end bound to be inclusive
            sendUdpRAW(message.substring(startIndex,endIndex+1));

            //start at the next character
            startIndex = endIndex+1;
        } while (endIndex != message.length() - 1);//terminate if we have reached the end
    }


    /**
     * This is a prate method to actually send a message over the udp protocol
     * @param message the message you wish to send
     */
    private void sendUdpRAW(String message){
        try(DatagramSocket serverSocket = new DatagramSocket()){
            DatagramPacket datagramPacket = new DatagramPacket(
                    message.getBytes(),
                    message.length(),
                    InetAddress.getByName("127.0.0.1"),//194"),
                    clientPort);

            serverSocket.send(datagramPacket);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    //These are the double buffering system
    private String lastUpdate = "";
    private String currentUpdate = "";

    /**
     * This will queue a message for sending, utilizing the double buffer
     * @param string the message you wish to send
     */
    public void addMessage(String string){
        //depending on the state of the semaphore we can do two things
        if(!sendLock.tryAcquire()){
            //if it is being used, set the last update
            lastUpdate = string;
        }else{
            //we can update the current update if we got past the semaphore
            currentUpdate = string;
            //release the semaphore since we have acquired
            sendLock.release();
        }
    }
}
