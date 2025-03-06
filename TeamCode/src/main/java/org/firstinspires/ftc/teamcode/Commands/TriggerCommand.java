package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TriggerCommand extends CommandBase {
    public TriggerReader leftTriggerReader, rightTriggerReader; //Readers for left and right trigger output values.

    public static boolean leftTriggerPressed = false, rightTriggerPressed = false; //Booleans for keeping track of the triggers' states.
    Telemetry telemetry;

    public TriggerCommand(GamepadEx gamepadEx, Telemetry telemetry) {
        this.telemetry = telemetry; //Setting up telemetry.
        leftTriggerReader = new TriggerReader(gamepadEx, GamepadKeys.Trigger.LEFT_TRIGGER); //Setting up trigger readers.
        rightTriggerReader = new TriggerReader(gamepadEx, GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    @Override
    public void execute() {
       if(leftTriggerPressed) { //Run a certain subsystem depending on which trigger was pressed.
           telemetry.addData("Left Trigger", "was just pressed.");
           telemetry.update();
       } else if(rightTriggerPressed) {
            telemetry.addData("Right trigger", "was just pressed.");
            telemetry.update();
       }
    }

    @Override
    public boolean isFinished() { //Ends the command depending on if the trigger pressed has now been released.
        return leftTriggerPressed ? leftTriggerReader.wasJustReleased()
                : rightTriggerReader.wasJustReleased();
    }

    @Override
    public void end(boolean interrupted) { //Depending on which trigger has been pressed and released, resets their boolean state back to normal.
        if(leftTriggerPressed && !leftTriggerReader.isDown()) {
            leftTriggerPressed = false;
        } else if(rightTriggerPressed && !rightTriggerReader.isDown()) {
            rightTriggerPressed = false;
        }
    }
}
