// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class DriveConstants {
        public static final int leftFrontID = 2;
        public static final int leftRearID = 3;
        public static final int rightFrontID = 13;
        public static final int rightRearID = 12;
        public static final double stickDB = 0.05;
        public static final double turnPow = 1.2;
        public static final double speedPow = 1.2;

        // drive tick to in = 1/ticks per rev * GR * wheel dia * pi
        public static final double driveTickToIN = 0.001020223994;
    }

    public static final class IndexerConstants {
        public static final int sensorThresh = 1;
        public static final int blueThresh = 100;
        public static final int redThresh = 100; 
        public static final int frontID = 6;
        public static final int rearID = 8;

    }

    public static final class IntakeConstants {
        public static final int frontID = 7;
        public static final int rearID = 9;

    }

    public static final class PNConstants {
        public static final int compressorID = 0;
        public static final int frontForwardID = 1;
        public static final int frontReverseID = 2;
        public static final int rearForwardID = 3;
        public static final int rearReverseID = 4;

    }

    public static final class ShooterConstants {
        public static final int leftMotorID = 10;
        public static final int rightMotorID = 1;

        public static final double shooterkP = 0.1;
        public static final double shooterkI = 0.0;
        public static final double shooterkD = 0.0001;
        public static final double shooterkF = 0;

        public static final double quadAimA = -0.269;
        public static final double quadAimB = 62.4;
        public static final double quadAimC = 2154;

    }

    public static final class HoodConstants {
        public static final int hoodID = 4;

        public static final double targetDistToSetpoint = 0;
        public static final float hoodForwardLimit = 5;
        public static final float hoodReverseLimit = 0;
        public static final double AccInPerSec = 10; 
        public static final double InPerSec = 19.59;
        public static final double kP = .05;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0; 
        public static final double kG = 0.02;
        public static final double kV = 0.05;
        public static final double kA = 0; 

        public static final double quadAimA = -0.00212;
        public static final double quadAimB = 0.418;
        public static final double quadAimC = -17;

    }

    public static final class TurretConstants {
        public static final int turretID = 11;
        // tickstodeg = turret pulley GR * 360/ticks per rev
        public static final double turretTicksToDegrees = 1.72193877551;
        public static final float turretForwardLimit = 70;
        public static final float turretReverseLimit = 70;
        public static final double turretkP = 0.16;
        public static final double turretkI = 0;
        public static final double turretkD = 0.00105;

    }

    public static final class VisionConstants {
        public static final double cameraHeightMETERS = Units.inchesToMeters(37.65);
        public static final double targetHeightMETERS = Units.inchesToMeters(104);
        public static final double cameraAngleRAD = Units.degreesToRadians(29);
    }
}
