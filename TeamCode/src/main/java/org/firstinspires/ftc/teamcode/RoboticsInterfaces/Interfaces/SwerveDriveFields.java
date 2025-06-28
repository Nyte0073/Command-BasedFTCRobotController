package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwerveDriveFields {

    public volatile IMU imu;
    public volatile double forwardVector;
    public volatile int normalizedHeading, totalHeadingFrontLeft, reversedHeadingFrontLeft,
    totalHeadingFrontRight, reversedHeadingFrontRight, totalHeadingBackLeft,
    reversedHeadingBackLeft, totalHeadingBackRight, reversedHeadingBackRight, previousTurningVector;
    public volatile boolean[] headingsReversed = new boolean[4], headingsNegativeOrNot = new boolean[4], wheelsHaveRotated = new boolean[4];
    public volatile int[] individualWheelHeadings = {0, 0, 0, 0}, individualTargetPositions = new int[4], previousTargetPositions = new int[4];
    public volatile boolean previousTurningLeft = false, fieldOriented, alreadyRotated;

    public volatile GamepadEx gamepadEx;
    public static volatile boolean stopMotorsIsRunning = false;
    public volatile Motor[] turningMotors, drivingMotors;

    public volatile Telemetry telemetry;
}
