// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int leftFrontID = 1;
        public static final int leftRearID = 2;
        public static final int rightFrontID = 3;
        public static final int rightRearID = 4;

        public static final double NEOticksToDegrees = 8.57142857143;
        public static final double driveReduction = 10; 
        public static final double wheelRadius = 3; 
    }

    public static final class IndexerConstants {
        public static final int frontID = 11;
        public static final int rearID = 12;

    }
    
    public static final class IntakeConstants {
        public static final int frontID = 5;
        public static final int rearID = 6;

    }

    public static final class PNConstants {
        public static final int compressorID = 0;
        public static final int frontForwardID = 1;
        public static final int frontReverseID = 2;
        public static final int rearForwardID = 3;
        public static final int rearReverseID = 4;

    }

    public static final class ShooterConstants {
        public static final int leftMotorID = 7;
        public static final int rightMotorID = 8;

        public static final double shooterkP = 0;
        public static final double shooterkI = 0;
        public static final double shooterkD = 0;
        public static final double shooterkF = 0;

    }

    public static final class HoodConstants {
        public static final int hoodID = 10;

        public static final double hoodkP = 0;
        public static final double hoodkI = 0;
        public static final double hoodkD = 0;
        public static final double hoodkF = 0;

    }

    public static final class TurretConstants {
        public static final int turretID = 9;

        public static final double turretkP = 0;
        public static final double turretkI = 0;
        public static final double turretkD = 0;
        public static final double turretkF = 0;

    }
    public static final class VisionConstants {
        public static final double cameraHeightMETERS = Units.inchesToMeters(37.65);
        public static final double targetHeightMETERS = Units.inchesToMeters(104); 
        public static final double cameraAngleRAD = Units.degreesToRadians(29);
    }
}
