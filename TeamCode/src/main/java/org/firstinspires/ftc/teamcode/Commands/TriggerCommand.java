package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainTeamcode.Teleop;

public class TriggerCommand extends CommandBase {
    public static boolean leftTriggerPressed = false, rightTriggerPressed = false; //Booleans for keeping track of the triggers' states.
    Telemetry telemetry;
    boolean leftTriggerDown, rightTriggerDown;

    public TriggerCommand(Telemetry telemetry) {
        this.telemetry = telemetry; //Setting up telemetry.
    }

    @Override
    public void execute() {
        leftTriggerDown = Teleop.leftReader.isDown();
        rightTriggerDown = Teleop.rightReader.isDown();
       if(leftTriggerPressed) { //Run a certain subsystem depending on which trigger was pressed.
           telemetry.addData("Left Trigger", "was just pressed.");
           telemetry.update();
       } else if(rightTriggerPressed) {
            telemetry.addData("Right trigger", "was just pressed.");
            telemetry.update();
       }
    }

    @Override
    public boolean isFinished() {
        return leftTriggerPressed ? !leftTriggerDown :
                !rightTriggerDown;
    }

    @Override
    public void end(boolean interrupted) {
        if(leftTriggerPressed) {
            leftTriggerPressed = false;
        } else if(rightTriggerPressed) {
            rightTriggerPressed = false;
        }
    }
}
