package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

public class DifferentialVector {

    /**Reference to the human driver's Xbox controller.*/
    public GamepadEx gamepadEx;

    /**The left and right motor groups of the differential drivetrain. Think of a motor group
     * like a bunch of motors working together in the same way, performing the exact same tasks. Because they are doing the
     * same thing, they are considered to be one motor, thus a motor group combines multiple motors into one motor that does only one action at a
     * time.*/
    public MotorGroup leftGroup, rightGroup;

    /**The type of differential drivetrain that is desired for the robot to drive with.*/
    String driveType;

    /**Constructs a new {@code DifferentialVector()} with an initialized {@code GamepadEx}, two {@code MotorGroup}'s,
     * and {@code driveType} String field.*/
    public DifferentialVector(GamepadEx gamepadEx, MotorGroup leftGroup, MotorGroup rightGroup, String driveType) {
        this.gamepadEx = gamepadEx;
        this.leftGroup = leftGroup;
        this.rightGroup = rightGroup;
        this.driveType = driveType;
    }
}
