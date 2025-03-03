package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Otos_Mecanum.Drawing;
import org.firstinspires.ftc.teamcode.Otos_Mecanum.SparkFunOTOSDrive;

public class LocalizationTest extends LinearOpMode {
    Servo leftEx, rightEX, gear;
    DcMotor elevator;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftEx = hardwareMap.get(Servo.class, "leftEx");
        rightEX = hardwareMap.get(Servo.class, "rightEX");
        gear = hardwareMap.get(Servo.class, "gear");
        elevator = hardwareMap.get(DcMotor.class, "erect");

//        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(41.5, 63, Math.PI));

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(41.5, 63, Math.toRadians(180)));
//        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        leftEx.setDirection(Servo.Direction.REVERSE);

//        leftEx.setPosition(Constants.ServoConstants.maxExtension);
//        rightEX.setPosition(Constants.ServoConstants.maxExtension);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("Elevator Position", elevator.getCurrentPosition());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
