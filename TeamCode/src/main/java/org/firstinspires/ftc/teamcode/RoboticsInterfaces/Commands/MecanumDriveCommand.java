package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.MecanumDrive;

public class MecanumDriveCommand extends CommandBase {

    /**The {@code MecanumDrive} subsystem that this command is going to control.*/
    MecanumDrive mecanumDrive;

    /**Reference to the human driver's Xbox controller.*/
    GamepadEx gamepadEx;

    /**The IMU orientation system of the robot. It returns the current heading that the robot is facing.*/
    IMU imu;

    /**Constructs a new {@code MecanumDriveCommand()} with an initialized {@code MecanumDrive} subsystem,
     * {@code GamepadEx} and {@code IMU}.*/
    public MecanumDriveCommand(MecanumDrive mecanumDrive, GamepadEx gamepadEx, IMU imu) {
        this.mecanumDrive = mecanumDrive;
        this.gamepadEx = gamepadEx;
        this.imu = imu;
    }

    /**When this method runs, it will stop all the motors of the mecanum drive if the human driver presses
     * the "A" button, and if he/she presses the "B" button instead, the robot will reset its gyro angular orientation, to make it think that it's at
     * a heading of 0 degrees again.*/
    @Override
    public void execute() {
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            mecanumDrive.stop();
        } else if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
            mecanumDrive.resetGyro(imu);
        } else {
            mecanumDrive.drive();
        }
    }
}
