package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TriggerCommand extends CommandBase {
    public TriggerReader leftTriggerReader, rightTriggerReader;

    public static boolean leftTriggerPressed = false, rightTriggerPressed = false;
    Telemetry telemetry;

    public TriggerCommand(GamepadEx gamepadEx, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftTriggerReader = new TriggerReader(gamepadEx, GamepadKeys.Trigger.LEFT_TRIGGER);
        rightTriggerReader = new TriggerReader(gamepadEx, GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    @Override
    public void execute() {
       if(leftTriggerPressed) {
           telemetry.addData("Left Trigger", "was just pressed.");
           telemetry.update();
       } else if(rightTriggerPressed) {
            telemetry.addData("Right trigger", "was just pressed.");
            telemetry.update();
       }
    }

    @Override
    public boolean isFinished() {
        return leftTriggerPressed ? leftTriggerReader.wasJustReleased()
                : rightTriggerReader.wasJustReleased();
    }

    @Override
    public void end(boolean interrupted) {
        if(leftTriggerPressed) {
            leftTriggerPressed = false;
        }
    }
}
