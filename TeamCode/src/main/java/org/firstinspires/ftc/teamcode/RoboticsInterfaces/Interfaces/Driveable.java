package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public interface Driveable {
    void stop();
    void updateTelemetry(Telemetry telemetry, Map<String, Object> telemetryMap);
    void drive();
    void resetGyro(IMU imu);
}
