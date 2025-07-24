package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.Mecanum;
import java.util.HashMap;

/**Class to implement when wanting to use a mecanum-type drivetrain for your robot. This class provides methods to drive the robot
 * based on input information from the {@code GamepadEx} controller and your orientation returned from your {@code IMU} to correctly steer
 * the robot in the right direction automatically, and also has methods to return information regarding the robot's current state back to
 * your {@code Telemetry} for you to be able to understand what the robot is doing on the field in real time.*/
public class MecanumDrive extends Mecanum {

    /**Reference to the robot's mecanum drivetrain, controlling our four mecanum wheels.*/
    com.arcrobotics.ftclib.drivebase.MecanumDrive mecanumDrive;

    /**Part of the four references to all the mecanum wheel hardware of your mecanum drivetrain.*/
    Motor frontLeft, frontRight, backLeft, backRight;

    /**Your information system. You can use it to return information back to the Driver Station screen to view certain things
     * about the robot in real time.*/
    Telemetry telemetry;

    /**Your robot's orientation system. Returns the current orientation of the robot in either degrees or radians, depending on your
     * choice.*/
    IMU imu;

    /**The boolean state of whether the robot will drive in a field-oriented way.*/
    boolean fieldOriented;

    /**Reference to the human driver's Xbox controller.*/
    GamepadEx gamepadEx;

    /**Constructs a new {@code MecanumDrive()} with an initialized {@code Motor[]}, {@code Telemetry}, {@code IMU},
     * {@code fieldOriented} field and {@code GamepadEx}*/
    public MecanumDrive(Motor[] motors, Telemetry telemetry, IMU imu, boolean fieldOriented, GamepadEx gamepadEx) {
        frontLeft = motors[0];
        frontRight = motors[1];
        backLeft = motors[2];
        backRight = motors[3];
        mecanumDrive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        this.telemetry = telemetry;
        this.imu = imu;
        this.fieldOriented = fieldOriented;
        this.gamepadEx = gamepadEx;
    }

    /**Stops the robot's motors by setting their motor powers to 0.*/
    @Override
    public void stopMotors() {
        mecanumDrive.stop();
    }

    /**Returns the instance of the {@code MecanumDrive} object being used to reference the robot's mecanum drivetrain, which is
     * going to be used to initialize code within the {@code Mecanum} superclass itself when calling its {@code drive()} method.*/
    @Override
    public com.arcrobotics.ftclib.drivebase.MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    /**Returns the {@code fieldOriented} variable that was initialized within the class's constructor method. This variable is going to
     * be used to toggle whether the {@code drive()} method in the {@code Mecanum} superclass will make the robot drive field-oriented
     * or robot-oriented.*/
    @Override
    public boolean getFieldOriented() {
        return fieldOriented;
    }

    /**Returns the {@code gamepadEx} field that was initialized within the class's constructor method. This field will be used in the
     * {@code Mecanum} superclass to get input from the human driver so that the program knows exactly what controller number input to
     * be using when making the robot drive.*/
    @Override
    public GamepadEx getGamepadEx() {
        return gamepadEx;
    }

    /**Returns the {@code imu} field from this class that is going to be used in the {@code Mecanum} superclass to*/
    @Override
    public IMU getIMU() {
        return imu;
    }

    @Override
    public void periodic() {
        updateTelemetry(telemetry, new HashMap<String, Object>() {{
            put("IMU Orientation", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }});
    }
}
