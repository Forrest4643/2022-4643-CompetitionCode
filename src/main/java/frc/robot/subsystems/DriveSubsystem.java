// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import net.thefletcher.revrobotics.CANSparkMax;
import net.thefletcher.revrobotics.enums.MotorType;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;

import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {

  // defining motor names and CAN ID's
  private final CANSparkMax leftLeader = new CANSparkMax(DriveConstants.leftFrontID, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(DriveConstants.leftRearID, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(DriveConstants.rightFrontID, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(DriveConstants.rightRearID, MotorType.kBrushless);

  // defining encoders
  public RelativeEncoder m_leftEncoder = leftLeader.getEncoder();
  public RelativeEncoder m_rightEncoder = rightLeader.getEncoder();

  // Defining drivetrain DifferentialDrive
  public final DifferentialDrive m_robotDrive = new DifferentialDrive(leftLeader, rightLeader);

  SlewRateLimiter driveSlew = new SlewRateLimiter(DriveConstants.driveSlew);
  SlewRateLimiter turnSlew = new SlewRateLimiter(DriveConstants.turnSlew);

  private AnalogGyro m_gyro = new AnalogGyro(1);
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  //Creates DriveSubsystem
  public DriveSubsystem() {
    // motor inversions
    leftLeader.setInverted(false);
    rightLeader.setInverted(true);

    // setting leftFront to follow leftRear,
    // and rightFront to follow rightRear
    leftFollower.follow(leftLeader, false);
    rightFollower.follow(rightLeader, false);

    // Defining simulated vision target
    simVision.addSimVisionTarget(
        new SimVisionTarget(farTargetPose, VisionConstants.targetGroundHeightM,
            VisionConstants.targetWidthM, VisionConstants.targetHeightM));

    // sending simulated field data to SmartDashboard
    SmartDashboard.putData("Field", m_field);
  } // end Public DriveSubsystem



  private Field2d m_field = new Field2d();

  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      new Rotation2d(-m_gyro.getAngle()), new Pose2d(6.383, 5.769, new Rotation2d()));

  Pose2d farTargetPose = new Pose2d(new Translation2d(VisionConstants.tgtXPos, VisionConstants.tgtYPos),
      new Rotation2d(0.0));

  // Creating a simulated photonvision system
  SimVisionSystem simVision = new SimVisionSystem(
      "photonvision",
      VisionConstants.camDiagFOV,
      VisionConstants.cameraAngleRAD,
      new Transform2d(),
      VisionConstants.cameraHeightM,
      VisionConstants.maxLEDRange,
      VisionConstants.camResolutionWidth,
      VisionConstants.camResolutionHeight,
      VisionConstants.minTargetAreaPIX);

  // Create the simulation model of our drivetrain.
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
      7.29, // 7.29:1 gearing reduction.
      7.5, // MOI of 7.5 kg m^2 (from CAD model).
      60.0, // The mass of the robot is 60 kg.
      Units.inchesToMeters(3), // The robot uses 3" radius wheels.
      0.7112, // The track width is 0.7112 meters.

      // The standard deviations for measurement noise:
      // x and y: 0.001 m
      // heading: 0.001 rad
      // l and r velocity: 0.1 m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)); // end m_driveSim

  public void DriveSiminit() {
    // this runs once at the start of Simulation
  }

  // reset DriveTrain encoders to 0
  public void resetDriveEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {

    // pass telemetry data to get Odometry data
    m_odometry.update(m_gyro.getRotation2d(),
        m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition());
    m_field.setRobotPose(m_odometry.getPoseMeters());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
  } // end void periodic

  @Override
  public void simulationPeriodic() {
    // Sets the sim inputs, -1 to 1 signal multiplied by robot controller voltage
    m_driveSim.setInputs(leftLeader.get() * RobotController.getInputVoltage(),
        rightLeader.get() * RobotController.getInputVoltage());

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);

    // update photonvision simulation
    simVision.processFrame(m_driveSim.getPose());

    // sending simulated encoder values to the main robot code
    m_leftEncoder.setPosition(m_driveSim.getLeftPositionMeters());
    m_rightEncoder.setPosition(m_driveSim.getRightPositionMeters());

    // Debug info
    // System.out.println("leftDriveDist:" + m_leftEncoder.getPosition());
    // System.out.println("rightDriveDist:" + m_rightEncoder.getPosition());

    // sending simulated gyro heading to the main robot code
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

  } // end simulationPeriodic

  // main setDrive void, this is used for the StickDrive command in TeleOp
  public void setDrive(double Speed, double turnRate) {

    double SqrTurn = DriveConstants.turnSin * (Math.sin(turnRate));

    double SqrSpeed = DriveConstants.speedSin * (Math.sin(Speed));

    // this ensures that negative inputs yield negative outputs,
    // and vise versa
    if (Speed < 0) {
      SqrSpeed = SqrSpeed * -1;
    }
    if (turnRate < 0) {
      SqrTurn = SqrTurn * -1;
    }

    m_robotDrive.arcadeDrive(driveSlew.calculate(SqrSpeed), turnSlew.calculate(SqrTurn) / 1.5);

    // debug info
    SmartDashboard.putNumber("sqrturn", SqrTurn);
    SmartDashboard.putNumber("sqrspeed", SqrSpeed);
  } // end setDrive

  // sets drive motors to a given voltage
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    m_robotDrive.feed();
  }

  // this is used for path-following.
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  // returns the average distance travelled between the left and right wheels
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  // resets the onboard gyro to read 0 degrees
  public void zeroHeading() {
    m_gyro.reset();
  }

  // returns the heading in degrees of the onboard gyro
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  // returns the turn rate of the onboard gyro
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  // returns the read position of the robot on the field
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // resets the read position of the robot on the field
  public void resetOdometry(Pose2d pose) {
    resetDriveEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
}