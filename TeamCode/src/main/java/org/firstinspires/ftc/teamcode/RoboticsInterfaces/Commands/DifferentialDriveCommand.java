package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.DifferentialDrive;

public class DifferentialDriveCommand extends CommandBase {
    DifferentialDrive differentialDrive;

    GamepadEx gamepadEx;

    IMU imu;

    boolean speedToggler = false;

    public DifferentialDriveCommand(DifferentialDrive differentialDrive, IMU imu, GamepadEx gamepadEx) {
        this.differentialDrive = differentialDrive;
        this.gamepadEx = gamepadEx;
        this.imu = imu;
    }

    @Override
    public void execute() {
        com.arcrobotics.ftclib.drivebase.DifferentialDrive differentialDrive1 = differentialDrive.getDifferentialDrive(
                differentialDrive.getDifferentialVector()
        );

        if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            differentialDrive.resetGyro(imu);
        } else if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)){
            speedToggler = !speedToggler;
            differentialDrive1.setMaxSpeed(speedToggler ? 0.5 : 1);
        } else {
            differentialDrive.drive();
        }
    }
}
