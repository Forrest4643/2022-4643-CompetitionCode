// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
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

  //defining encoders
  private RelativeEncoder m_leftFrontEncoder = leftFront.getEncoder();
  private RelativeEncoder m_leftRearEncoder = leftRear.getEncoder();
  private RelativeEncoder m_rightFrontEncoder = rightFront.getEncoder();
  private RelativeEncoder m_rightRearEncoder = rightRear.getEncoder();


  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftDrive, rightDrive);

  private AnalogGyro m_gyro = new AnalogGyro(1);
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  private Field2d m_field = new Field2d();

  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    new Rotation2d(m_gyro.getAngle()), new Pose2d(5.0, 13.5, new Rotation2d()));

  // Create the simulation model of our drivetrain.
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
    7.29,                    // 7.29:1 gearing reduction.
    7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
    60.0,                    // The mass of the robot is 60 kg.
    Units.inchesToMeters(3), // The robot uses 3" radius wheels.
    0.7112,                  // The track width is 0.7112 meters.

    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
  );

  public void DriveSiminit() {
    REVPhysicsSim.getInstance().addSparkMax(leftFront, DCMotor.getNEO(2));
    REVPhysicsSim.getInstance().addSparkMax(rightFront, DCMotor.getNEO(2));
  }

  public double getDriveDistanceIN() {
    // returns the average position of all drive encoders.
    double driveForwardRAW = ((m_leftFrontEncoder.getPosition() + m_leftRearEncoder.getPosition()) / 2)
        + ((m_rightFrontEncoder.getPosition() + m_rightRearEncoder.getPosition()) / 2) / 2;

    return driveForwardRAW * DriveConstants.driveTickToIN;
  }

  public void resetDriveEncoders() {
    m_leftFrontEncoder.setPosition(0);
    m_leftRearEncoder.setPosition(0);
    m_rightFrontEncoder.setPosition(0);
    m_rightRearEncoder.setPosition(0);
  }

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    leftDrive.setInverted(false);
    rightDrive.setInverted(true);
    SmartDashboard.putData("Field", m_field);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("DriveDistanceIN", getDriveDistanceIN());
    m_odometry.update(m_gyro.getRotation2d(),
    m_leftFrontEncoder.getPosition(),
    m_rightFrontEncoder.getPosition());
    m_field.setRobotPose(m_odometry.getPoseMeters());

    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
  }


  @Override
  public void simulationPeriodic() {
    //Sets the sim inputs, -1 to 1 signal multiplied by robot controller voltage
    m_driveSim.setInputs(leftDrive.get() * RobotController.getInputVoltage(), rightDrive.get() * RobotController.getInputVoltage());

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);

    REVPhysicsSim.getInstance().run();

    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

  }  

  public void setDrive(double Speed, double turnRate) {

    // inputs to a power for a nice response curve

    double SqrSpeed = Math.pow(MathUtil.applyDeadband(Math.abs(Speed), DriveConstants.stickDB), DriveConstants.speedPow);
    double SqrTurn = Math.pow(MathUtil.applyDeadband(Math.abs(turnRate), DriveConstants.stickDB), DriveConstants.turnPow);

    if (Speed < 0) {
      SqrSpeed = SqrSpeed * -1;
    }

    if (turnRate < 0) {
      SqrTurn = SqrTurn * -1;
    }

    m_robotDrive.arcadeDrive(SqrSpeed, SqrTurn);

    SmartDashboard.putNumber("sqrturn", SqrTurn);
    SmartDashboard.putNumber("sqrspeed", SqrSpeed);
  }
}

