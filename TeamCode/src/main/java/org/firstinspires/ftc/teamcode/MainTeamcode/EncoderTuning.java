package org.firstinspires.ftc.teamcode.MainTeamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Encoder Tuning", group = "teamcode")
public class EncoderTuning extends LinearOpMode {
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            double leftFrontPosition = leftFront.getCurrentPosition();
            double rightFrontPosition = rightFront.getCurrentPosition();
            double leftBackPosition = leftBack.getCurrentPosition();
            double rightBackPosition = rightBack.getCurrentPosition();

            double averagePosition = (leftBackPosition + leftFrontPosition + rightBackPosition + rightFrontPosition) / 4;

            telemetry.addData("Position", averagePosition);
            telemetry.update();
        }
    }
}
