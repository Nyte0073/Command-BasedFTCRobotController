package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import android.util.Log;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.ThreadBasedSwerveDrive;

import java.io.ObjectOutputStream;
import java.net.ServerSocket;
import java.net.Socket;

public class BackEndServer extends Thread {
    private final int serverPort;
    private final RobotLog.ThreadBasedSwerveLog threadBasedSwerveLog = new RobotLog.ThreadBasedSwerveLog();
    private final ThreadBasedSwerveDrive swerveDrive = new ThreadBasedSwerveDrive();
    private Object[] line = null;
    public BackEndServer(int serverPort, String name) {
        super(name);
        this.serverPort = serverPort;
    }
    public void launchServer() throws Exception {
        ServerSocket serverSocket = new ServerSocket(serverPort);
        Socket socket = serverSocket.accept();
        ObjectOutputStream outputStream = new ObjectOutputStream(socket.getOutputStream());
        Log.i(getClass().getName(), "Socket connected.");

        while(!Thread.currentThread().isInterrupted()) {
            setLine();
            if(line != null) {
                outputStream.writeObject(line);
                outputStream.flush();
                line = null;
            }
            Thread.sleep(10);
        }
    }

    public void setLine() {
        this.line = threadBasedSwerveLog.setThreadBasedSwerveLog(
                swerveDrive.getRobotBooleanArrays(), swerveDrive.getForwardVectorAndNormalizedHeading(),
                swerveDrive.getRobotBooleans()
        );
    }

    @Override
    public void run() {
        try {
            launchServer();
        } catch(Exception e) {
            Thread.currentThread().interrupt();
        }
    }
}