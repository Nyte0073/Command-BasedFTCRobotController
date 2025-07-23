package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.Differential;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.DifferentialVector;

/**Class containing the methods for running a smooth differential drive, either in a tank or arcade style,
 * depending on what kind of driving the human driver wants their robot to have.*/
public class DifferentialDrive extends Differential {

    /**Reference to the human driver's Xbox controller.*/
    GamepadEx gamepadEx;

    /**Motor group object that contains {@code Motor} objects for either the left or right side of the robot.*/
    MotorGroup leftGroup, rightGroup;

    /**The type of drivetrain the robot will use.*/
    String driveType;

    /**Constructs a new {@code DifferentialDrive()} with an initialized {@code GamepadEx}, {@code MotorGroup}'s and
     * {@code driveType} field.*/
    public DifferentialDrive(GamepadEx gamepadEx, Motor[] motors, String driveType) {
        this.gamepadEx = gamepadEx;
        leftGroup = new MotorGroup(motors[0], motors[2]);
        rightGroup = new MotorGroup(motors[1], motors[3]);
        this.driveType = driveType;
    }

    /**Returns the vector class that contains al the variables for initializing the drivetrain and running the {@code drive()} method.*/
    @Override
    public DifferentialVector getDifferentialVector() {
        return new DifferentialVector(gamepadEx, leftGroup, rightGroup, driveType);
    }

    /**Stops the drivetrain's motors by setting their motor powers to 0.*/
    @Override
    public void stopMotors() {
        leftGroup.stopMotor();
        rightGroup.stopMotor();
    }
}
