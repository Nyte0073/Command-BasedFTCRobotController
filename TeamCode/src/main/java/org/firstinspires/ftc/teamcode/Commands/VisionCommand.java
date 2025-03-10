package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.MainTeamcode.Teleop;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;

public class VisionCommand extends CommandBase {
    public static boolean isRange = false;
    Vision vision;
    static States state;

    public VisionCommand(Vision vision, States state) {
        this.vision = vision;
        VisionCommand.state = state;
        addRequirements(vision);
    }

    public static States getState() {
        return state;
    }

    @Override
    public void execute() {
        if(isRange) {
            state = States.IN_RANGE;
        }

       switch(state) {
           case IN_RANGE:
               break;

           case NOT_IN_RANGE:
               break;

           case IN_RANGE_BUT_NOT_ALIGNED:
               break;

           case IN_RANGE_BUT_NOT_CLOSE_ENOUGH:
               break;
       }
    }

    public enum States {
        IN_RANGE,
        NOT_IN_RANGE,
        IN_RANGE_BUT_NOT_ALIGNED,
        IN_RANGE_BUT_NOT_CLOSE_ENOUGH
    }
}
