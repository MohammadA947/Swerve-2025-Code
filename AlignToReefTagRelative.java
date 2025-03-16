// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain m_drivetrain;
  private SwerveRequest.FieldCentricFacingAngle m_drive;
  private Pose2d m_targetPose = new Pose2d();
  private double tagID = -1; //-1

  private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric();
  private double positions[];
  private double xSpeed = xController.calculate(positions[2]);
  private double ySpeed = -yController.calculate(positions[0]);
  private double rotValue = -rotController.calculate(positions[4]);
  


  public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain m_drivetrain) {
    xController = new PIDController(AprilConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(AprilConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(AprilConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.m_drivetrain = m_drivetrain;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(AprilConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(AprilConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(AprilConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(AprilConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? AprilConstants.Y_SETPOINT_REEF_ALIGNMENT : -AprilConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(AprilConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      positions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", positions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double xSpeed = xController.calculate(positions[2]);
      double ySpeed = -yController.calculate(positions[0]);
      double rotValue = -rotController.calculate(positions[4]);
    
      m_drivetrain.setControl(robotDrive
      .withVelocityX(xSpeed)
      .withVelocityY(ySpeed)
      .withRotationalRate(rotValue)
      );
    //   m_drivetrain.setControl(m_drive
    //     .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    //     .withVelocityX(xSpeed) //  Drive forward with negative Y (forward)
    //     .withVelocityY(ySpeed) //  Drive left with negative X (left)
    //     .withTargetDirection(m_targetPose.getRotation())); //  Replace with rotValue
    

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
        m_drivetrain.setControl(robotDrive
      .withVelocityX(xSpeed)
      .withVelocityY(ySpeed)
      .withRotationalRate(rotValue)
      );
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    System.out.println(positions);
  }

  @Override
  public void end(boolean interrupted) {
    // m_drivetrain.setControl(m_drive
    //     .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    //     .withVelocityX(xSpeed) //  Drive forward with negative Y (forward)
    //     .withVelocityY(ySpeed) //  Drive left with negative X (left)
    //     .withTargetDirection(m_targetPose.getRotation()));
    m_drivetrain.setControl(robotDrive
      .withVelocityX(xSpeed)
      .withVelocityY(ySpeed)
      .withRotationalRate(rotValue)
    );
    //  m_drivetrain.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(AprilConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(AprilConstants.POSE_VALIDATION_TIME);
  }
}