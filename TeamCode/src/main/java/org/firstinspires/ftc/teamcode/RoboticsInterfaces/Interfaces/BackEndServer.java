package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import android.util.Log;
import com.google.gson.Gson;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.ThreadBasedSwerveDrive;

import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;

public class BackEndServer extends Thread {

    /**The port used to connect the {@code BackEndServer} and {@code RobotGUIUpdater} classes together over a network.*/
    private final int serverPort;

    /**Reference to the {@code ThreadBasedSwerveDrive} object being used to return all the information about the robot's current state.*/
    private final ThreadBasedSwerveDrive swerveDrive = new ThreadBasedSwerveDrive();

    /**Reference to the {@code RobotLog.ThreadBasedSwerveLog} class that is responsible for cloning all the robot information and
     * gathering it all into one class to send to the {@code RobotGUIUpdater} class in IntelliJ IDEA all at once.*/
    private final RobotLog.ThreadBasedSwerveLog swerveLog = new RobotLog.ThreadBasedSwerveLog();

    /**Constructs a new {@code BackEndServer()} with an initialized {@code serverPort} field.*/
    public BackEndServer(int serverPort) {
        this.serverPort = serverPort;
    }

    /**Launches the server to start sending information to the {@code RobotGUIUpdater} class for it to start updating. This method also
     * starts up the server that the {@code RobotGUIUpdater} class is then going to connect to. Word of caution: this method has to be started
     * before the {@code RobotGUIUpdater} class calls its {@code launchGUIServer()} method, because this method is responsible for creating the server
     * for {@code RobotGUIUpdater} to connect to.*/
    public void launchServer() throws Exception {
        ServerSocket serverSocket = new ServerSocket(serverPort);
        Socket socket = serverSocket.accept();
        Log.i(getClass().getSimpleName(), "Socket connected");
        Gson gson = new Gson();
        OutputStream outputStream = socket.getOutputStream();

        while(!Thread.currentThread().isInterrupted()) {
            setLog();
            String convertedToJson = gson.toJson(swerveLog);
            outputStream.write(convertedToJson.getBytes());
            outputStream.flush();
            Thread.sleep(10);
        }
    }

    /**Creates clones of all the information stored in the {@code ThreadBasedSwerveDrive} class about the robot and declares it all
     * within itself, keeping the information all together.*/
    public void setLog() {
        swerveLog.setThreadBasedSwerveLog(swerveDrive.getRobotBooleanArrays(), swerveDrive.getForwardVectorAndNormalizedHeading(),
                swerveDrive.getRobotBooleans(), swerveDrive.getTargetPositions());
    }

    /**Launches the server.*/
    @Override
    public void run() {
        try {
            launchServer();
        } catch(Exception e) {
            Thread.currentThread().interrupt();
        }
    }
}