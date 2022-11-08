// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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

        public static final double drivekP = 0;
        public static final double drivekI = 0;
        public static final double drivekD = 0;

        public static final double steerkP = 0;
        public static final double steerkI = 0;
        public static final double steerkD = 0;

        public static final double highGoal = 12 * 6;
        public static final double lowGoal = 12 * 2;

        //TODO sysid
        public static final double driveTickToIN = -22.28169203;
        public static final double kaVoltSecondsSquaredPerMeter = 1;
        public static final double ksVolts = 0.001;
        public static final double kvVoltSecondsPerMeter = 1;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.7112);
        public static final double kPDriveVel = 0.05;
    }

    public static final class IndexerConstants {
        public static final double sensorThresh = 150;
        public static final double blueThresh = .33;
        public static final double redThresh = .33;
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

        public static final double shooterkP = 0.0001;
        public static final double shooterkI = 0.0004;
        public static final double shooterkD = 0.0;
        public static final double shooterkF = 0;

        public static final double quadAimA = -0.269;
        public static final double quadAimB = 62.4;
        public static final double quadAimC = 2154;

    }

    public static final class HoodConstants {
        public static final int hoodID = 4;

        public static final double conversionFactor = 2.513274123;
        public static final double targetDistToSetpoint = 0;
        public static final float hoodForwardLimit = 5;
        public static final float hoodReverseLimit = 0;
        public static final double AccInPerSec = 10;
        public static final double InPerSec = 19.59;
        public static final double kP = .2;
        public static final double kI = 0.02;
        public static final double kD = 0.001;
        public static final double kS = 0;
        public static final double kG = 0.02;
        public static final double kV = 0.05;
        public static final double kA = 0;

        public static final double lowGoal = 3;
        public static final double highGoal = 0;

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
        public static final double cameraHeightM = Units.inchesToMeters(37.650);
        public static final double targetGroundHeightM = Units.inchesToMeters(104.000);
        public static final double cameraAngleRAD = Units.degreesToRadians(30.0);
        public static final double camDiagFOV = 170.0; // degrees - assume wide-angle camera
        public static final double maxLEDRange = 20.0; // meters
        public static final int camResolutionWidth = 640; // pixels
        public static final int camResolutionHeight = 480; // pixels
        public static final double minTargetAreaPIX = 10; // square pixels

      
        public static final double targetWidthM = Units.inchesToMeters(53.750);
      
        public static final double targetHeightM = Units.inchesToMeters(2.000);
      
        public static final double tgtXPos = Units.inchesToMeters(312.000);
        public static final double tgtYPos = Units.inchesToMeters(164.000);
               
    }

    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;

    }
}
