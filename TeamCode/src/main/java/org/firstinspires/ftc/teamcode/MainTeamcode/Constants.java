package org.firstinspires.ftc.teamcode.MainTeamcode;

/**Class containing the constant variables for all the subsystems, so that each subsystem has to be designed in a manner
 * that fits within the constraints of the constants.*/
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

    public static class HolonomicConstants {

        /**The motor angle constants for a three wheel holonomic drive. You can changes these depending on the configuration of your
         * holonomic drive, just make sure to ONLY change it here, in the {@code MotorConstants.HolonomicConstants} class and no where else.*/
        public static final double[] motorAngles = {60, 120, 270};
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

        /**Magic number for indicating that no reversal should happen for a motor.*/
        public static final int NO_REVERSAL = 2000;
    }

    public static class ServoConstants {
        /**The name of the robot's claw servo.*/
        public static final String clawServo = "claw";
    }
}
