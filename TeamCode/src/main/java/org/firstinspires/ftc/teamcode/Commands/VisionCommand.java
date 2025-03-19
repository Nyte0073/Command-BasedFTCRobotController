package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;

public class VisionCommand extends CommandBase {

    /**Reference to the robot's entire mainframe for vision processing.*/
    Vision vision;

    /**The state of the robot in terms of position relative to its desired target.*/
    public static States state;

    Telemetry telemetry;

    /**Constructs a new {@code VisionCommand()} with an initialized {@code Vision()} and
     * {@code State()}.*/
    public VisionCommand(Vision vision, States state, Telemetry telemetry) {
        this.vision = vision;
        this.telemetry = telemetry;
        VisionCommand.state = state;
        addRequirements(vision);
    }

    /**Returns the state of the robot's position relative to desired target.*/
    public static States getState() {
        return state;
    }

    @Override
    public void execute() {
       switch(state) {
           case IN_RANGE:
               telemetry.addData("Robot", "is in range.");
               break;

           case NOT_IN_RANGE:
               telemetry.addData("Robot", "is not in range.");
               break;

           case IN_RANGE_BUT_NOT_ALIGNED:
               telemetry.addData("Robot", "is in range but not aligned.");
               break;

           case ALIGNED_BUT_NOT_CLOSE_ENOUGH:
               telemetry.addData("Robot", "is aligned but not close enough.");
               break;
       }
    }

    /**Positional states directly telling where the robot is in relative to where it wants
     * to be in terms of reaching its target object in the desired place.*/
    public enum States {

        /**The robot is in range and aligned with its target.*/
        IN_RANGE,

        /**The robot is not in range and not aligned with its target.*/
        NOT_IN_RANGE,

        /**The robot is in range with its target but not aligned with it.*/
        IN_RANGE_BUT_NOT_ALIGNED,

        /**The robot is aligned with its target but not in range (not close enough to it vertically).*/
        ALIGNED_BUT_NOT_CLOSE_ENOUGH
    }
}
