package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public abstract class Swerve extends SubsystemBase implements Driveable {
    double forwardPower, sidePower, turningVector;
    int headingInDegrees;
    boolean fieldOriented, turningLeft;
    @Override
    public void stop() {
        stopMotors();
    }

    @Override
    public void resetGyro(IMU imu) {
        imu.resetYaw();
    }

    @Override
    public void drive() {
        forwardPower = getRobotDoubles()[0];
        sidePower = getRobotDoubles()[1];
        turningVector = getRobotDoubles()[2];
        headingInDegrees = getHeadingInDegrees();

        fieldOriented = getFieldOriented();
        turningLeft = getTurningLeft();

        setSwerveModuleState(fieldOriented, forwardPower, sidePower, headingInDegrees, turningVector, turningLeft);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry, Map<String, Object> telemetryMap) {
        for(String s : telemetryMap.keySet()) {
            telemetry.addData(s, telemetryMap.get(s));
        }
    }

    public abstract void setSwerveModuleState(boolean fieldOriented, double forwardPower, double sidePower, int headingInDegrees, double turningVector, boolean turningLeft);

    public abstract void stopMotors();

    public abstract void resetWheelHeading();

    public abstract void completeRotate(boolean turningLeft, int imuHeadingInDegrees, double turnVector, boolean fieldOriented);

    public abstract void setPowerForCompleteRotate(boolean turningLeft, boolean[] headingsReversed, double turningVector, int[] targetPositions, boolean goToPosition);

    public abstract int normalizeHeading(int currentPosition, int targetPosition);

    public abstract boolean getFieldOriented();

    public abstract boolean getTurningLeft();

    public abstract double[] getRobotDoubles();

    public abstract int getHeadingInDegrees();

    public abstract void setPower(boolean[] headingsReversed, double forwardVector);
}
