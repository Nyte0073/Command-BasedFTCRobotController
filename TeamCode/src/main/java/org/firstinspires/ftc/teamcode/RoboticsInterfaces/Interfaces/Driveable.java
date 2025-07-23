package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Map;

public interface Driveable {

    /**Stops the robot.*/
    void stop();

    /**Updates the robot's {@code Telemetry} object with current information.*/
    void updateTelemetry(Telemetry telemetry, Map<String, Object> telemetryMap);

    /**Drives the robot in a specific way according to the specified drive type and drivetrain type.*/
    void drive() throws Exception;

    /**Resets the robot's IMU by setting its orientation reading back to 0 degrees.*/
    void resetGyro(IMU imu);
}


