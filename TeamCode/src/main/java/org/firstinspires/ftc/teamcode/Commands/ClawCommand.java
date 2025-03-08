package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;

public class ClawCommand extends CommandBase {

    /**The robot's claw.*/
    private final Claw claw;

    /**The driver's controller.*/
    private final GamepadEx gamepadEx;

    /**Constructs a new {@code ClawCommand} with an initialized {@code Claw} and {@code GamepadEx}.*/
    public ClawCommand(Claw claw, GamepadEx gamepadEx) {
        this.claw = claw;
        this.gamepadEx = gamepadEx;

        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.rotateClawByDegrees(360 / gamepadEx.getLeftX());
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

}
