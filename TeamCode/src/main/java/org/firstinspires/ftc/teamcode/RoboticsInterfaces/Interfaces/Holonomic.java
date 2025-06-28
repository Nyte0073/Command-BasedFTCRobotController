package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import java.util.Map;

/**Class to extend when wanting to make a Holonomic drivetrain. This class contains methods for
 * updating the robot's telemetry, stopping the robot's motors in case of emergency, resetting the robot's IMU,
 * making the robot drive, and provides blueprint abstract methods for extending classes to fill in, to make sure that this class
 * gets the information fields it needs to complete certain tasks for the robot to drive.*/
public abstract class Holonomic extends SubsystemBase implements Driveable {

    /**Stops the robot's motors.*/
    @Override
    public void stop() {
        stopMotors();
    }

    /**Updates the robot's telemetry all at once using a {@code Map<String, Object>} to gather all the data from the robot and store, and
     * then update the robot's {@code telemetry} object with it by looping through the map's contents.*/
    @Override
    public void updateTelemetry(Telemetry telemetry, Map<String, Object> telemetryMap) {
        for(String s : telemetryMap.keySet()) {
            telemetry.addData(s, telemetryMap.get(s));
        }
    }

    /**Resets the IMU orientation of the robot back to 0, so
     * that the robot now thinks its current heading is 0 degrees.*/
    @Override
    public void resetGyro(IMU imu) {
        imu.resetYaw();
    }

    /**Drives the robot in either a field-oriented or non field-oriented way depending on the value of the
     * {@code fieldOriented} boolean returned from any extending classes.*/
    @Override
    public void drive() {
        String driveType = getDriveType();
        List <Motor[]> hMotors = getHolonomicMotors();
        double[] motorAngles = getHolonomicMotorAngles();

        HolonomicVector holonomicVector = new HolonomicVector(hMotors.get(0), hMotors.get(1), motorAngles[0],
                motorAngles[1], motorAngles[2], driveType);

        HDrive drive = holonomicVector.drivetrain;
        boolean fieldOriented = getFieldOriented();
        GamepadEx gamepadEx = getGamepadEx();
        IMU imu = getIMU();

        if(fieldOriented) {
           drive.driveFieldCentric(gamepadEx.getLeftX(), -gamepadEx.getLeftY(), gamepadEx.getRightX(),
                   imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        } else {
            drive.driveRobotCentric(gamepadEx.getLeftX(), -gamepadEx.getLeftY(), gamepadEx.getRightX());
        }
    }

    /**Returns the type of drivetrain you want the robot to behave like.*/
    public abstract String getDriveType();

    /**Returns the {@code Motor} arrays for the Holonomic class
     *  to initialize its {@code HolonomicVector} object.*/
    public abstract List<Motor[]> getHolonomicMotors();

    /**Returns the angles (in decimals) of the motors depending on
     * where on the robot they are positioned.*/
    public abstract double[] getHolonomicMotorAngles();

    /**Returns whether the robot should drive field-oriented or not.*/
    public abstract boolean getFieldOriented();

    /**Returns the {@code gamepadEx} field from other extending subclasses.*/
    public abstract GamepadEx getGamepadEx();

    /**Returns the {@code imu} field from other extending subclasses.*/
    public abstract IMU getIMU();

    /**Sets the power of all the driving motors on the robot to 0, in case
     * there is an emergency and the robot needs to stop right away and the
     * Xbox controller won't respond.*/
    public abstract void stopMotors();
}
