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
        public static final int leftFrontID = 2;
        public static final int leftRearID = 3;
        public static final int rightFrontID = 13;
        public static final int rightRearID = 12;
        
        //drive tick to in = 1/ticks per rev * GR * wheel dia * pi
        public static final double driveTickToIN = (((1.0/42) * 10) * 6) * Math.PI;
    }

    public static final class IndexerConstants {
        public static final int frontID = 11;
        public static final int rearID = 12;

    }
    
    public static final class IntakeConstants {
        public static final int frontID = 0;
        public static final int rearID = 8;

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
        //tickstodeg = turret pulley GR * 360/ticks per rev
        public static final double turretTicksToDegrees = (224/30) * (360/4096);
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
