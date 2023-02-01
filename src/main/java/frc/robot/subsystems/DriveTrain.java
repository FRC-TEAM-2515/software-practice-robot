package frc.robot.subsystems;

/** 
 *  Imports robotics and WPILib specific classes
 * 
 * Add an import for "MotorControllerGroup"
 */
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** 
 * Imports classes from other package by following the file path
 * 
 * 
 * Add an import for "RobotContainer"  
 */
import frc.robot.commands.*;
import frc.robot.Constants.DriveConstants;

/**
 *  Imports classes that allows us to interface with the robot's motors and electronics 
 * 
 *  Usually from vendor libraries
 * 
 *  Add an import for the WPI_VictorSPX motor and an import for an Xbox controller
 */
 import com.ctre.phoenix.motorcontrol.NeutralMode;
 import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
 import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
 import com.ctre.phoenix.motorcontrol.FeedbackDevice;
 import com.kauailabs.navx.frc.AHRS;

 public class DriveTrain extends SubsystemBase {
    /**
     * Declares (names) the components used in the class
     */
   
    // The motors on the left side of the drive.
    private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
          new WPI_TalonSRX(DriveConstants.kLeftLeaderPort),
          new WPI_VictorSPX(DriveConstants.kLeftFollowerPort));

    // Declare the motors on the right side of the drive. Name the MotorControllerGroup "m_rightMotors"

    // protected XboxController driveController;
    // static final double turnGain = DriveConstants.kTurnGain;
    // static final double deadband = DriveConstants.kDeadband;
    // static final double driveGain = DriveConstants.kDriveGain;
    // private final DifferentialDriveOdometry odometry;

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // The gyro sensor
    private final AHRS ahrs;


    /** Creates a new DriveSubsystem. */
    public DriveTrain Subsystem() {

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. 

      /** 
       * Invert Motor? and set Break Mode
       * 
       * Repeat with left side motors except they are not inverted
       */
      m_rightMotors.setInverted(true);
      m_rightMotors.setNeutralMode(NeutralMode.Coast);



    
    /* Configure Sensor */
        // Phase sensor to have positive increment when driving Talon Forward (Green
        // LED)
        m_rightMotors.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        m_rightMotors.setSensorPhase(true);

        m_leftMotors.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        m_leftMotors.setSensorPhase(false);

    // The robot's drive
     final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftMotors.reset();
    m_rightMotors.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftMotors.getDistance() + m_rightMotors.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftMotors() {
    return m_leftMotors;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightMotors() {
    return m_rightMotors;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    ahrs.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
    public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-ahrs.getAngle());
    }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
    public double getTurnRate() {
    return -ahrs.getRate();
  }
}

