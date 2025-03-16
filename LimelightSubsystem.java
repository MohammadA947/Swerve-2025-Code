package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.RobotContainer;
import static edu.wpi.first.units.Units.*;

import javax.naming.spi.DirObjectFactory;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


 


public class LimelightSubsystem extends SubsystemBase {
    double tx = LimelightHelpers.getTX("");
    private double xSpeed;
    private double ySpeed;
    private double rot;
    private boolean fieldRelative;

    public LimelightSubsystem()
    {
      limelight_aim_proportional();
      limelight_range_proportional();

    }
     // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);  ;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  } 
  
   public void limelightOverride()
    {
        final var rot_limelight = limelight_aim_proportional();
        rot = rot_limelight;
        System.out.println("This changed!!!!");

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;

        //while using Limelight, turn off field-relative driving.
        fieldRelative = false;
    }

    
  public double getxSpeed() {
    return xSpeed;
  }

  public double getySpeed() {
    return ySpeed;
  }

  public double getrot() {
    return rot;
  }
}