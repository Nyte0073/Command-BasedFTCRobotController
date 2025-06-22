package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.Differential;
import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Interfaces.DifferentialVector;

public class DifferentialDrive extends Differential {

    GamepadEx gamepadEx;
    MotorGroup leftGroup, rightGroup;
    String driveType;

    public DifferentialDrive(GamepadEx gamepadEx, Motor[] motors, String driveType) {
        this.gamepadEx = gamepadEx;
        leftGroup = new MotorGroup(motors[0], motors[2]);
        rightGroup = new MotorGroup(motors[1], motors[3]);
        this.driveType = driveType;
    }

    @Override
    public DifferentialVector getDifferentialVector() {
        return new DifferentialVector(gamepadEx, leftGroup, rightGroup, driveType);
    }

    @Override
    public void stopMotors() {
        leftGroup.stopMotor();
        rightGroup.stopMotor();
    }
}
