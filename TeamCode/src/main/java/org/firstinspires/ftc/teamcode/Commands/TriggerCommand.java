package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainTeamcode.Teleop;

public class TriggerCommand extends CommandBase {

    /**Boolean state for one of the driver's controller's triggers being pressed or not.*/
    public static boolean leftTriggerPressed = false, rightTriggerPressed = false;

    /**The robot's telemetry system.*/
    Telemetry telemetry;

    /**Boolean state for when one of the driver's triggers is pressed down or not.*/
    boolean leftTriggerDown, rightTriggerDown;

    /**Constructs a new TriggerCommand() with initialized {@code telemetry}.*/
    public TriggerCommand(Telemetry telemetry) {
        this.telemetry = telemetry; //Setting up telemetry.
    }

    @Override
    public void execute() {
        leftTriggerDown = Teleop.leftReader.isDown();
        rightTriggerDown = Teleop.rightReader.isDown();

       if(leftTriggerPressed) { //Run a certain subsystem depending on which trigger was pressed.
           telemetry.addData("Left Trigger", "was just pressed.");
       } else if(rightTriggerPressed) {
            telemetry.addData("Right trigger", "was just pressed.");
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
