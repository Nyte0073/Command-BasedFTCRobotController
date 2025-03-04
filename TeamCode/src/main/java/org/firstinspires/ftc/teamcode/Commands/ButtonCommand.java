package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ButtonCommand extends CommandBase {
    private final GamepadEx gamepadEx; //Driver's controller.
    GamepadKeys.Button button; //A button on the driver's controller.
    Telemetry telemetry; //Source of output information returned from the robot. Essentially the same as SmartDashboard.
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
