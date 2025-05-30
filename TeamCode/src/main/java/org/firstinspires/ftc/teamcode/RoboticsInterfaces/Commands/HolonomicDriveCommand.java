package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.HolonomicDrive;

public class HolonomicDriveCommand extends CommandBase {
    HolonomicDrive holonomicDrive;

    IMU imu;

    GamepadEx gamepadEx;

    public HolonomicDriveCommand(HolonomicDrive holonomicDrive, IMU imu, GamepadEx gamepadEx) {
        this.holonomicDrive = holonomicDrive;
        this.imu = imu;
        this.gamepadEx = gamepadEx;
    }

    @Override
    public void execute() {
        holonomicDrive.drive();
    }
}
