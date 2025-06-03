package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public abstract class Holonomic extends SubsystemBase implements Driveable {
    @Override
    public void drive() {
        RobotVector robotVector = getRobotVector();
        int headingInDegrees = robotVector.imuHeadingInDegrees;
        double forwardPower = robotVector.forwardPower;
        double sidePower = robotVector.sidePower;
        double turningVector = robotVector.turningVector;
        boolean turningLeft = robotVector.turningLeft;
        boolean fieldOriented = robotVector.fieldOriented;

        setPower(headingInDegrees, fieldOriented, forwardPower, sidePower, turningVector, turningLeft);
    }

    @Override
    public void resetGyro(IMU imu) {
        imu.resetYaw();
    }

    @Override
    public void stop() {
        stopMotors();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry, Map<String, Object> telemetryMap) {
        for(String s : telemetryMap.keySet()) {
            telemetry.addData(s, telemetryMap.get(s));
        }
    }

    public abstract void setPower(int headingInDegrees, boolean fieldOriented, double forwardPower, double sidePower, double turningVector, boolean turningLeft);

    public abstract void stopMotors();

    public abstract RobotVector getRobotVector();
}
