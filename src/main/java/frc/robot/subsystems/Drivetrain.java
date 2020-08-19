/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */

  AHRS gyro = new AHRS(SPI.Port.kMXP);

  Pose2d startingPose = new Pose2d(5.0, 13.5, new Rotation2d());

  Pose2d pose;

  CANSparkMax leftFront = new CANSparkMax(Constants.LEFT_FRONT, MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(Constants.LEFT_BACK, MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax(Constants.RIGHT_FRONT, MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(Constants.RIGHT_BACK, MotorType.kBrushless);
  
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.TRACK_WIDTH));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), startingPose);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka);

  PIDController leftPIDController = new PIDController(Constants.kp, 0, 0);
  PIDController rightPIDController = new PIDController(Constants.kp, 0, 0);
  
  public Drivetrain() {
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    leftFront.setInverted(false);
    rightFront.setInverted(true);

  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftFront.getEncoder().getVelocity() / Constants.DRIVE_GEAR_RATIO * 2 * 
      Math.PI * Units.inchesToMeters(Constants.DRIVE_WHEEL_RADIUS) / 60,
      rightFront.getEncoder().getVelocity() / Constants.DRIVE_GEAR_RATIO * 2 * 
      Math.PI * Units.inchesToMeters(Constants.DRIVE_WHEEL_RADIUS) / 60
    );
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }
  
  public Pose2d getPose(){
    return pose;
  }

  public void setOutput(double leftVolts, double rightVolts) {
    leftFront.set(leftVolts / 12);
    rightFront.set(rightVolts / 12);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), getSpeeds().leftMetersPerSecond, getSpeeds().rightMetersPerSecond);
  }
}
