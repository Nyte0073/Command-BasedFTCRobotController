package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

/**Class containing all abstract methods and functionality to produce a fully functioning field-oriented or robot-oriented swerve
 * drivetrain for FTC. When extending this class, you will have to implement multiple abstract methods to give your swerve drive the
 functionality you want, and depending on how you want your swerve drive to function, you could also create your own methods alongside the methods
 given to you by this class, to help make things cleaner and more organized.*/
public abstract class Swerve extends SubsystemBase implements Driveable {

    /**Forward joystick input.*/
    double forwardPower,

    /**Side joystick input.*/
            sidePower,

    /**Turning vector movement of the robot.*/
            turningVector;

    /**The IMU orientation of the robot, in degrees.*/
    int headingInDegrees;

    /**State of the whether the robot is driven field-oriented or robot-oriented.*/
    boolean fieldOriented,

    /**State of whether the robot is rotating left or not.*/
            turningLeft;

    /**Stops the robot's motors.*/
    @Override
    public void stop() {
        stopMotors();
    }

    /**Resets the yaw angle of the robot's IMU system.*/
    @Override
    public void resetGyro(IMU imu) {
        imu.resetYaw();
    }

    /**Drives the robot either in a field-oriented or non field-oriented way.
     * Automatically calls {@code setSwerveModuleState()} to drive the robot and updates
     * the state of all the robot's driving components so that the robot drives in the new updated way according to the
     * human driver.*/
    @Override
    public void drive() throws Exception {
        SwerveVector robotVector = getRobotVector();

        forwardPower = robotVector.forwardPower;
        sidePower = robotVector.sidePower;
        turningVector = robotVector.turningVector;
        headingInDegrees = robotVector.imuHeadingInDegrees;

        fieldOriented = robotVector.fieldOriented;
        turningLeft = robotVector.turningLeft;

        setSwerveModuleState(fieldOriented, forwardPower, sidePower, headingInDegrees, turningVector, turningLeft);
    }

    /**Updates the telemetry all at once by calling {@code telemetry.addData()} on everything inside the {@code telemetryMap}
     * variable, so that you won't have to be calling {@code telemetry.addData()} on EVERY SINGLE thing in the code multiple times to get a
     * reading on what's going on inside the subsystem.*/
    @Override
    public void updateTelemetry(Telemetry telemetry, Map<String, Object> telemetryMap) {
        for(String s : telemetryMap.keySet()) {
            telemetry.addData(s, telemetryMap.get(s));
        }
    }

    /**Updates the state of all the turning and driving motors, from an at-rest position (if the user input all equals 0) or a driving/rotating
     * position (depending on if the user push the left or right joystick certain directions). If the user input corresponds to driving, the robot
     * will drive depending on the state of the {@code fieldOriented} boolean parameter, which will ensure that the robot either drives field-oriented
     * or robot-oriented.*/
    public abstract void setSwerveModuleState(boolean fieldOriented, double forwardPower, double sidePower, int headingInDegrees, double turningVector, boolean turningLeft) throws Exception;

    /**Stops the turning and driving motors of the robot by setting their motor powers to 0.*/
    public abstract void stopMotors();

    /**Resets the wheel heading of all the wheels to current heading of the robot, so that the wheels are now able to turn in the same motion
     * when just driving and not rotating on the spot with different headings. The wheels can now drive with the same heading.*/
    public abstract void resetWheelHeading();

    /**Calculates the headings for all the swerve wheels to turn either 45 or -45 degrees. By making the front-left/back-right wheels turn
     * -45 degrees, and the front-right/back-left wheels turn 45 degrees, you can make the robot rotate completely on the spot by making the motor
     * power of the first motor pair the opposite of the motor powers of the second motor pair.*/
    public abstract void completeRotate(boolean turningLeft, int imuHeadingInDegrees, double turnVector, boolean fieldOriented);

    /**Sets the power of the turning/driving motors once the {@code completeRotate()} method finishes calculating the headings for all the individual
     * wheels. This method will set motor powers specific to if the headings of some of the wheels are reversed, and also if the robot is rotating left
     * or right.*/
    public abstract void setPowerForCompleteRotate(boolean turningLeft, boolean[] headingsReversed, double turningVector, int[] targetPositions, boolean goToPosition, boolean[] headingsDirectionsNegative);

    /**Normalizes the difference between the target heading and current heading of the robot, so that it can take the shortest angle relative to its
     * current heading to get to the point on the Cartesian plane that the target heading points in the direction of.*/
    public abstract int normalizeHeading(int currentPosition, int targetPosition);

    /**Returns the RobotVector (user input information) about the robot.*/
    public abstract SwerveVector getRobotVector();


    /**Sets the power of the turning and driving wheels of the robot when driving normally at a certain angle.*/
    public abstract void setPower(boolean[] headingsReversed, double forwardVector);
}
