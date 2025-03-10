package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;

public class Vision extends SubsystemBase {
    public HuskyLens huskyLens;
    HuskyLens.Block block;
    Telemetry telemetry;

    public Vision(HuskyLens huskyLens, Telemetry telemetry) {
        this.huskyLens = huskyLens;
        this.telemetry = telemetry;

        telemetry.addData("HuskyLens available", huskyLens.knock());
    }

    public boolean hasBlock() {
        if(huskyLens.blocks().length > 0) {
            block = huskyLens.blocks()[0];
        }
        return huskyLens.blocks().length != 0;
    }

    @Override
    public void periodic() {
        telemetry.addLine("HUSKY LENS DATA");
        telemetry.addData("Husky Lens Block Count", huskyLens.blocks().length);
        telemetry.addData("Husky Lens First Piece Info", hasBlock() ? huskyLens.blocks()[0].toString() : "HuskyLens doesn't have a piece.");

        if(!VisionCommand.isRange && (block == null ? -1 : block.y) > 0 && (hasBlock() ? block.y : 0) < 100) {
            VisionCommand.isRange = true;
        }
    }
}
