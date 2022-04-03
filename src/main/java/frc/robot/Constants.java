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
        public static final double turnPow = 1;
        public static final double speedPow = 1.2;

        public static final double drivekP = 0.03;
        public static final double drivekI = 0.004;
        public static final double drivekD = 0;

        public static final double bangTol = .25;

        public static final double steerkP = 0.05;
        public static final double steerkI = 0.0008;
        public static final double steerkD = 0.005;

        public static final double driveSlew = 1;
        public static final double turnSlew = 5;

        public static final double highGoal = 12 * 6;
        public static final double lowGoal = 12 * 2;

        public static final double autoDist = -2 * 12;

        public static final double driveTickToIN = -22.28169203;
    }

    public static final class IndexerConstants {

       
        public static final double thresh1 = 1;
        public static final double frontThresh = 0;
        public static final double rearThresh = 0;
        public static final double blueThresh = .33;
        public static final double redThresh = .33;
        public static final double oneBall = 20;
        public static final double primeShot = -10;
        public static final double bangTolerance = .75;
        public static final int frontID = 6;
        public static final int rearID = 8;

    }

    public static final class IntakeConstants {
        public static final int frontID = 7;
        public static final int rearID = 9;

    }

    public static final class PNConstants {
        public static final int compressorID = 0;
        public static final int frontForwardID = 2;
        public static final int frontReverseID = 1;
        public static final int rearForwardID = 4;
        public static final int rearReverseID = 3;

    }

    public static final class ShooterConstants {
        public static final int leftMotorID = 10;
        public static final int rightMotorID = 1;

        public static final double lowGoal = 2000;
        public static final double highGoal = 5600;
        public static final double ejectCargo = 1000;

        public static final double kP = 0.015;
        public static final double kI = 0;
        public static final double kD = 0.0004;
        public static final double PIDtolerance = 10;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kS = 8.85;
        public static final double acc = 3746;
        public static final double responseTimeS = 0.04;

        public static final double quadAimA = -0.223;
        public static final double quadAimB = 10.5;
        public static final double quadAimC = -70.2;
        public static final double quadAimD = 1432;
        public static final double efficiencyConversion = .84;

    }

    public static final class HoodConstants {
        public static final int hoodID = 4;

        public static final double conversionFactor = 2.513274123;
        public static final float ForwardLimit = 3;
        public static final float ReverseLimit = 0;
        public static final double PIDtolerance = 0.05;

        public static final double kP = .2;
        public static final double kI = 0.02;
        public static final double kD = 0.001;

        // public static final double AccInPerSec = 10;
        // public static final double InPerSec = 19.59;
        // public static final double kS = 0;
        // public static final double kG = 0.02;
        // public static final double kV = 0.05;
        // public static final double kA = 0;

        public static final double lowGoal = 3;
        public static final double highGoal = 0;

        public static final double quadAimA = -0.167;
        public static final double quadAimB = 5.11;
        public static final double quadAimC = 44;

    }

    public static final class TurretConstants {
        public static final int turretID = 11;
        // tickstodeg = turret pulley GR * 360/ticks per rev
        public static final double turretTicksToDegrees = 1.72193877551;
        public static final float turretForwardLimit = 270;
        public static final float turretReverseLimit = 0;
        public static final double turretkP = 0.16;
        public static final double turretkI = 0;
        public static final double turretkD = 0.00105;
        public static final double tolerance = .5;
        public static final double zeroThresh = 1;


    }

    public static final class VisionConstants {

        public static final double distA = 0.055;
        public static final double distB = 0.765;
        public static final double distC = -1.3;
        public static final double distD = 0;

        public static final double distanceOffset = .5;
        public static final double cameraHeightMETERS = Units.inchesToMeters(37.65);
        public static final double targetHeightMETERS = Units.inchesToMeters(104);
        public static final double cameraAngleRAD = Units.degreesToRadians(30);
    }
}
