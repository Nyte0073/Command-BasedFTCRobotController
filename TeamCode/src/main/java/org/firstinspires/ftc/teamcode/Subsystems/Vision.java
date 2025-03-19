package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;

public class Vision extends SubsystemBase {

    /**The robot's vision system.*/
    public HuskyLens huskyLens;

    /**Representation of something the Husky Lens sees.*/
    HuskyLens.Block block;

    /**The robot's telemetry system.*/
    Telemetry telemetry;

    /**Boolean state for keeping of whether the robot is range of its target
     * and/or if it's aligned with its target.*/
    static boolean aligned = false, inRange = false;

    /**Constructs a new {@code Vision()} with an initialized {@code HuskyLens}
     * and {@code Telemetry}.*/
    public Vision(HuskyLens huskyLens, Telemetry telemetry) {
        this.huskyLens = huskyLens;
        this.telemetry = telemetry;

        telemetry.addData("HuskyLens available", huskyLens.knock() ? "True" : "Problem communicating with HuskyLens.");
    }

    /**Returns whether the Husky Lens sees a piece or not.*/
    public boolean hasBlock() {
        return huskyLens.blocks().length != 0;
    }

    /**Updates the {@code telemetry} with data collected from the {@code huskyLens},
     * and also updates the state of the robot's proximity to the desired target.*/
    @Override
    public void periodic() {
//        if(huskyLens.blocks().length > 0) {
//            block = huskyLens.blocks()[0] == null ? null : huskyLens.blocks()[0];
//        }

        try {
            assert block != null;
            aligned = block.x < 176 && block.x > 169;
            inRange = block.y < 100;
        } catch (NullPointerException e) {
            telemetry.addData("Exception", "Block is null.");
        }

        if(aligned && inRange) {
            VisionCommand.state = VisionCommand.States.IN_RANGE;
        } else if(aligned) {
            VisionCommand.state = VisionCommand.States.ALIGNED_BUT_NOT_CLOSE_ENOUGH;
        } else if(inRange) {
            VisionCommand.state = VisionCommand.States.IN_RANGE_BUT_NOT_ALIGNED;
        } else {
           VisionCommand.state = VisionCommand.States.NOT_IN_RANGE;
        }

        telemetry.addLine("HUSKY LENS DATA");
        telemetry.addData("Husky Lens Block Count", huskyLens.blocks().length);
//        telemetry.addData("Husky Lens First Piece Info", hasBlock() ? huskyLens.blocks()[0].toString() : "HuskyLens doesn't have a piece.");
        telemetry.addData("Vision State", VisionCommand.getState());
    }
}
