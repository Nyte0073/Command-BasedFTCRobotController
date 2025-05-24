package org.firstinspires.ftc.teamcode.MainTeamcode;

public class Constants {
    public static class MotorConstants {
        /**Front-left mecanum drive motor/swerve turning motor..*/
        public static final String frontLeftMotor = "front_left";

        /**Front-right mecanum drive motor/swerve turning motor.*/
        public static final String frontRightMotor = "front_right";

        /**Back-left mecanum drive motor/swerve turning motor..*/
        public static final String backLeftMotor = "back_left";

        /**Back-right mecanum drive motor/swerve turning motor.*/
        public static final String backRightMotor = "back_right";
    }

    public static class SwerveConstants {

        /**Front-left swerve driving motor.*/
        public static final String frontLeftDriving = "frontLeftDriving",

        /**Front-right swerve driving motor.*/
        frontRightDriving = "frontRightDriving",

        /**Back-left swerve driving motor.*/
        backLeftDriving = "backLeftDriving",

        /**Back-right swerve driving motor.*/
        backRightDriving = "backRightDriving";

        /**Constant for normalizing {@code forwardVector} motor powers so they can only go to a
         * maximum absolute value of 1.*/
        public static final double vectorScalar = 1.4142135623730950488016887242097;
    }

    public static class ServoConstants {
        /**The name of the robot's claw servo.*/
        public static final String clawServo = "claw";
    }
}
