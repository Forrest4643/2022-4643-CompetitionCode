// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {

  // defining motor names
  private final CANSparkMax leftFront = new CANSparkMax(DriveConstants.leftFrontID, MotorType.kBrushless);
  private final CANSparkMax leftRear = new CANSparkMax(DriveConstants.leftRearID, MotorType.kBrushless);
  private final CANSparkMax rightFront = new CANSparkMax(DriveConstants.rightFrontID, MotorType.kBrushless);
  private final CANSparkMax rightRear = new CANSparkMax(DriveConstants.rightRearID, MotorType.kBrushless);

  // setting speed controller groups
  private final MotorControllerGroup leftDrive = new MotorControllerGroup(leftFront, leftRear);
  private final MotorControllerGroup rightDrive = new MotorControllerGroup(rightFront, rightRear);

  private RelativeEncoder leftFrontEncoder = leftFront.getEncoder();
  private RelativeEncoder leftRearEncoder = leftRear.getEncoder();
  private RelativeEncoder rightFrontEncoder = rightFront.getEncoder();
  private RelativeEncoder rightRearEncoder = rightRear.getEncoder();

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftDrive, rightDrive);

  SlewRateLimiter driveSlew = new SlewRateLimiter(DriveConstants.driveSlew);
  SlewRateLimiter turnSlew = new SlewRateLimiter(DriveConstants.turnSlew);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    leftDrive.setInverted(false);
    rightDrive.setInverted(true);
    leftFrontEncoder.setPositionConversionFactor(1);
    leftRearEncoder.setPositionConversionFactor(1);
    rightFrontEncoder.setPositionConversionFactor(1);
    rightRearEncoder.setPositionConversionFactor(1);
  }

  public double getDriveDistanceIN() {
    // returns the average position of all drive encoders.
    double driveForwardRAW = ((leftFrontEncoder.getPosition() + leftRearEncoder.getPosition()) / 2)
        + ((rightFrontEncoder.getPosition() + rightRearEncoder.getPosition()) / 2) / 2;

    return -((driveForwardRAW / 360) * 10) * (6.283185*3);
  }

  public void resetDriveEncoders() {
    leftFrontEncoder.setPosition(0);
    leftRearEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    rightRearEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("DriveDistanceIN", getDriveDistanceIN());
  }

  public void setDrive(double Speed, double turnRate) {

    // inputs to a power for a nice response curve

    //TODO added "speedSin"
    // double SqrSpeed = Math.pow(MathUtil.applyDeadband(Math.abs(Speed),
    // DriveConstants.stickDB),
    // DriveConstants.speedPow);

    double SqrTurn = Math.pow(MathUtil.applyDeadband(Math.abs(turnRate), DriveConstants.stickDB),
        DriveConstants.turnPow);

    double SqrSpeed = (Math.sin(Speed));

    if (Speed < 0) {
      SqrSpeed = Math.abs(SqrSpeed) * -1;
    }

    if (turnRate < 0) {
      SqrTurn = Math.abs(SqrTurn) * -1;
    }

    // TODO Update turnSlew to match the lower COM of the robot -F.L
    m_robotDrive.arcadeDrive(driveSlew.calculate(SqrSpeed), turnSlew.calculate(SqrTurn) / 1.5);

    SmartDashboard.putNumber("sqrturn", SqrTurn);
    SmartDashboard.putNumber("sqrspeed", SqrSpeed);
  }
}
