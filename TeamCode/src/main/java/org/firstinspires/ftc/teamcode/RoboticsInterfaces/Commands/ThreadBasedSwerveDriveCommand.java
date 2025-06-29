package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.ThreadBasedSwerveDrive;

public class ThreadBasedSwerveDriveCommand extends CommandBase {

    ThreadBasedSwerveDrive swerveDrive;
    GamepadEx gamepadEx;
    IMU imu;

    public ThreadBasedSwerveDriveCommand(ThreadBasedSwerveDrive swerveDrive, GamepadEx gamepadEx, IMU imu) {
        this.swerveDrive = swerveDrive;
        this.gamepadEx = gamepadEx;
        this.imu = imu;
    }

    @Override
    public void execute() {
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            swerveDrive.resetGyro(imu);
        } else if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
            swerveDrive.stopMotors();
            swerveDrive.stopThreads();
        }
    }
}
