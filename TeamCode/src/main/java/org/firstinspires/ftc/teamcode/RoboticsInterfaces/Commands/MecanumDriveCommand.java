package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.MecanumDrive;

public class MecanumDriveCommand extends CommandBase {
    MecanumDrive mecanumDrive;
    GamepadEx gamepadEx;
    IMU imu;
    public MecanumDriveCommand(MecanumDrive mecanumDrive, GamepadEx gamepadEx, IMU imu) {
        this.mecanumDrive = mecanumDrive;
        this.gamepadEx = gamepadEx;
        this.imu = imu;
    }

    @Override
    public void execute() {
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            mecanumDrive.stop();
        } else if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
            mecanumDrive.resetGyro(imu);
        }

        mecanumDrive.drive();
    }
}
