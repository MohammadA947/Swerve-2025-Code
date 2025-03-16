package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class Constants 
{

  public static final class ElevatorConstants 
  {
    public static final int kElevatorLeftMotorID = 9;        //Pratice Robot ID: 3
    public static final int kElevatorRightMotorID = 10;      //Pratice Robot ID: 44 
    public static final double kFirstPosition = 0;
    public static final double kSecondPosition = 9.2;             
    public static final double kThirdPosition = 26.5;
    public static final double kFourthPosition = 54;
  }

  public static final class AlgaeConstants
  {
    public static final int kAlgaeArmMotorID = 2; 
    public static final int kAlgaeIntakeMotorID = 3;

    public static final double kArmRestPosition = 0.0;
    public static final double kArmUpPosition = 17.4;
    public static final double kArmScorePosition = 5;
  }

  public static final class CoralConstants 
  {
    public static final double coralSpeed = 0.12;          //

    public static final int kCoralLeftMotorID = 4;         // Pratice Robot ID: 23
    public static final int kCoralRightMotorID = 6;        // Pratice Robot ID: 22

    public static final int kCoralEnterSensorID = 60;
    public static final int kCoralExitSensorID = 61;       

    public static final int kCoralEnterSensorLimit = 80;   // Pratice Robot ID: 12;
    public static final int kCoralExitSensorLimit = 60;

    public static final double kCoralFineTuner = 3;
  }

  public final class LEDConstants {
    public static final int CANdleID = 14;
    public static final int JoystickId = 0;

    public static final int MaxBrightnessAngle = 90;
    public static final int MidBrightnessAngle = 180;
    public static final int ZeroBrightnessAngle = 270;

    public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
    public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;

    public static final int BlockButton = XboxController.Button.kStart.value;

    public static final int VbatButton = XboxController.Button.kA.value;
    public static final int V5Button = XboxController.Button.kB.value;
    public static final int CurrentButton = XboxController.Button.kX.value;
    public static final int TemperatureButton = XboxController.Button.kY.value;
}

  public final class AprilConstants {
    public static final double X_REEF_ALIGNMENT_P = 0;
    public static final double Y_REEF_ALIGNMENT_P = 0;
    public static final double ROT_REEF_ALIGNMENT_P = 0;
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0;
    public static final double X_SETPOINT_REEF_ALIGNMENT = 0;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0;
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0;
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0;
    public static final double DONT_SEE_TAG_WAIT_TIME = 0.3;
    public static final double POSE_VALIDATION_TIME = 0;
    

  }




// public static final class AutoConstants 
//   {
//     public static final double kMaxSpeedMetersPerSecond = 2.0;
//     public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
//     public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
//     public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
//     // *********************************************
//     // *   MUST BE CHANGED BASED ON SYSID VALUES   *
//     // *********************************************
//     public static final double kPXController = 0.5;
//     public static final double kPYController = 0.5;
//     public static final double kPThetaController = 0;
//     // public static final double kPXController = 0.5;
//     // public static final double kPYController = 0.5;
//     // public static final double kPThetaController = 0;
//     // Constraint for the motion profilied robot angle controller
//     public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
//   }


}
