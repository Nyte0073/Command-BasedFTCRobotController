package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

public class DifferentialVector {

    public GamepadEx gamepadEx;

    public MotorGroup leftGroup, rightGroup;

    String driveType;

    public DifferentialVector(GamepadEx gamepadEx, MotorGroup leftGroup, MotorGroup rightGroup, String driveType) {
        this.gamepadEx = gamepadEx;
        this.leftGroup = leftGroup;
        this.rightGroup = rightGroup;
        this.driveType = driveType;
    }
}
