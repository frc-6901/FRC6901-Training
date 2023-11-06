// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N2;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class RomiDrivetrain extends SubsystemBase {

  // Romi Encoder Constants
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark mLeftMotor = new Spark(0);
  private final Spark mRightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder mLeftEncoder = new Encoder(4, 5);
  private final Encoder mRightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive mDiffDrive = new DifferentialDrive(mLeftMotor, mRightMotor);

  // Set up Xbox Controller
  final int controllerPort = 0;
  XboxController controller = new XboxController(controllerPort);


  // SIMULATION STUFF

  private final EncoderSim mLeftEncoderSim = new EncoderSim(mLeftEncoder);
  private final EncoderSim mRightEncoderSim = new EncoderSim(mRightEncoder);

  private final RomiGyro mGyro = new RomiGyro();
  private final LinearSystem<N2, N2, N2> mDrivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  private final DifferentialDrivetrainSim mDrivetrainSimulator =new DifferentialDrivetrainSim(mDrivetrainSystem, DCMotor.getCIM(2), 8, 0.787, 0.0762, null);

  private Field2d mField = new Field2d();

  private final DifferentialDriveOdometry mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));



  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    mLeftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    mRightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    SmartDashboard.putData("Field", mField);
    resetEncoders();
  }

  /**
   * Commands the drivetrain through an arcade drive style. HINT: this is useful for your purposes.
   * @param xaxisSpeed  The speed at which the robot goes forward and backward. Should be a number between -1 and 1.
   * @param zaxisRotate The speed at which the spins clockwise and counterclockwise. Should be a number between -1 and 1.
   */
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    mDiffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    mLeftEncoder.reset();
    mRightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return mLeftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return mRightEncoder.getDistance();
  }

  @Override
  public void simulationPeriodic() {
    mDrivetrainSimulator.setInputs(mLeftMotor.get() * RobotController.getInputVoltage(), mRightMotor.get() * RobotController.getInputVoltage());
    mDrivetrainSimulator.update(0.02);
    
    mLeftEncoderSim.setDistance(mDrivetrainSimulator.getLeftPositionMeters());
    mRightEncoderSim.setDistance(mDrivetrainSimulator.getRightPositionMeters());
    mLeftEncoderSim.setRate(mDrivetrainSimulator.getLeftVelocityMetersPerSecond());
    mRightEncoderSim.setRate(mDrivetrainSimulator.getRightVelocityMetersPerSecond());

    mGyro.setPosY(-mDrivetrainSimulator.getHeading().getDegrees());
  }

  @Override
  public void periodic() {
    mOdometry.update(Rotation2d.fromDegrees(mGyro.getAngleY()), mLeftEncoder.getDistance(), mRightEncoder.getDistance());
    mField.setRobotPose(mOdometry.getPoseMeters());
  }
}
