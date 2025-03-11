package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ButtonCommand extends CommandBase {
    /**The driver's controller.*/
    private final GamepadEx gamepadEx;

    /**Representation of a button on the driver's controller.*/
    GamepadKeys.Button button;

    /**The robot's telemetry system.*/
    Telemetry telemetry;

    /**The robot's gyroscope system.*/
    IMU imu;

    /**Constructs a new {@code ButtonCommand} with initialized {@code GamepadEx}, {@code GamepadEx.Button}'s,
     and {@code Telemetry}.*/
    public ButtonCommand(GamepadEx gamepadEx, GamepadKeys.Button button, Telemetry telemetry, IMU imu) {
        this.gamepadEx = gamepadEx;
        this.button = button;
        this.telemetry = telemetry;
        this.imu = imu;
    }

    @Override
    public void execute() { //Run a different subsystem depending on the type of button that was pressed.
        switch(button) {
            case DPAD_DOWN:
                telemetry.addData("DPAD_DOWN", "was just pressed.");
                break;

            case DPAD_UP:
                telemetry.addData("DPAD_UP", "was just pressed.");
                break;

            case DPAD_LEFT:
                telemetry.addData("DPAD_LEFT", "was just pressed.");
                break;

            case DPAD_RIGHT:
                telemetry.addData("DPAD_RIGHT", "was just pressed.");
                break;

            case A:
                telemetry.addData("A", "was just pressed, and yaw was reset.");
                imu.resetYaw();
                break;

            case B:
                telemetry.addData("B", "was just pressed.");
                break;

            case X:
                telemetry.addData("X", "was just pressed.");
                break;

            case Y:
                telemetry.addData("Y", "was just pressed.");
                break;

        }
    }

    @Override
    public boolean isFinished() {
        return gamepadEx.wasJustReleased(button);
    }
}
