package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwerveDriveFields {

    public volatile IMU imu;
    public volatile double forwardVector = 0;
    public volatile int normalizedHeading = 0, totalHeadingFrontLeft = 0, reversedHeadingFrontLeft = 0,
    totalHeadingFrontRight = 0, reversedHeadingFrontRight = 0, totalHeadingBackLeft = 0,
    reversedHeadingBackLeft = 0, totalHeadingBackRight = 0, reversedHeadingBackRight = 0, previousTurningVector = 0;
    public volatile boolean[] headingsReversed = new boolean[4], headingsNegativeOrNot = new boolean[4], wheelsHaveRotated = new boolean[4];
    public volatile int[] individualWheelHeadings = {0, 0, 0, 0}, individualTargetPositions = new int[4], previousTargetPositions = new int[4];
    public volatile boolean previousTurningLeft = false, fieldOriented = false, alreadyRotated = false;

    public volatile GamepadEx gamepadEx;
    public static volatile boolean stopMotorsIsRunning = false;
    public volatile Motor[] turningMotors, drivingMotors;

    public volatile Telemetry telemetry;
}
