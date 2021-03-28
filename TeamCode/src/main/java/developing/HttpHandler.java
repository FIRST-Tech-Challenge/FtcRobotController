package developing;

import java.net.*;
import java.io.*;

public class HttpHandler {
    // initialize socket and input output streams
    private Socket socket		 = null;
    private DataInputStream input = null;
    private DataOutputStream out	 = null;

    // constructor to put ip address and port
    public HttpHandler(String address, int port)
    {
        // establish a connection
        try
        {
            socket = new Socket(address, port);

            // takes input from terminal

            // sends output to the socket
            out = new DataOutputStream(socket.getOutputStream());
        } catch(IOException ignore) {}

        // string to read message from input
        String line = "";

        // keep reading until "Over" is input
        while (!line.equals("over"))
        {
            try
            {
                String test = "test";
                out.writeUTF(test);
            }
            catch(IOException i)
            {
            }
        }

        // close the connection
        try
        {
            input.close();
            out.close();
            socket.close();
        }
        catch(IOException i)
        {
        }
    }

}
