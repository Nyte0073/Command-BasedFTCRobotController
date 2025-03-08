package org.firstinspires.ftc.teamcode.Commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ButtonCommand extends CommandBase {
    /**The driver's controller.*/
    private final GamepadEx gamepadEx;

    /**Representation of a button on the driver's controller.*/
    GamepadKeys.Button button;

    /**The robot's telemetry system.*/
    Telemetry telemetry;

    /**Constructs a new {@code ButtonCommand} with initialized {@code GamepadEx}, {@code GamepadEx.Button}'s,
     and {@code Telemetry}.*/
    public ButtonCommand(GamepadEx gamepadEx, GamepadKeys.Button button, Telemetry telemetry) {
        this.gamepadEx = gamepadEx;
        this.button = button;
        this.telemetry = telemetry;
    }

    @Override
    public void execute() { //Run a different subsystem depending on the type of button that was pressed.
        switch(button) {
            case DPAD_DOWN:
                telemetry.addData("DPAD_DOWN", "was just pressed.");
                telemetry.update();
                break;

            case DPAD_UP:
                telemetry.addData("DPAD_UP", "was just pressed.");
                telemetry.update();
                break;

            case DPAD_LEFT:
                telemetry.addData("DPAD_LEFT", "was just pressed.");
                telemetry.update();
                break;

            case DPAD_RIGHT:
                telemetry.addData("DPAD_RIGHT", "was just pressed.");
                telemetry.update();
                break;

            case A:
                Log.i(FtcRobotControllerActivity.TAG, "A was pressed.");
                telemetry.addData("A", "was just pressed.");
                telemetry.update();
                break;

            case B:
                telemetry.addData("B", "was just pressed.");
                telemetry.update();
                break;

            case X:
                telemetry.addData("X", "was just pressed.");
                telemetry.update();
                break;

            case Y:
                telemetry.addData("Y", "was just pressed.");
                telemetry.update();
                break;

        }
    }

    @Override
    public boolean isFinished() {
        return gamepadEx.wasJustReleased(button);
    }
}
