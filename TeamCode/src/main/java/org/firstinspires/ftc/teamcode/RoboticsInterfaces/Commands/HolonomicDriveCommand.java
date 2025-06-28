package org.firstinspires.ftc.teamcode.RoboticsInterfaces.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoboticsInterfaces.Subsystems.HolonomicDrive;

public class HolonomicDriveCommand extends CommandBase {

    /**The {@code HolonomicDrive} subsystem that this command is going to control.*/
    HolonomicDrive holonomicDrive;

    /**The IMU orientation system of the robot. It returns the current heading the robot is facing.*/
    IMU imu;

    /**Reference to the human driver's Xbox controller.*/
    GamepadEx gamepadEx;

    /**Constructs a new {@code HolonomicDriveCommand()} with an initialized {@code HolonomicDrive} subsystem,
     * {@code IMU} and {@code GamepadEx}.*/
    public HolonomicDriveCommand(HolonomicDrive holonomicDrive, IMU imu, GamepadEx gamepadEx) {
        this.holonomicDrive = holonomicDrive;
        this.imu = imu;
        this.gamepadEx = gamepadEx;
    }

    /**This method will simply make the robot drive according to the input values from the human driver's Xbox controller.*/
    @Override
    public void execute() {
        holonomicDrive.drive();
    }
}
