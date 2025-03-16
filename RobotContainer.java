// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.ser.std.MapProperty;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.*;

//Import Subsystems
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CANdleSystem.AnimationTypes;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

//LEDs
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public final SwerveRequest.RobotCentric forwardstraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //Robot Subsystems
    private final CommandXboxController driveJoystick = new CommandXboxController(0);
    private final CommandXboxController opJoystick = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();
    private final CoralSubsystem m_Coral = new CoralSubsystem();
    private final CANdleSystem m_CANdle = new CANdleSystem(opJoystick);
    private final AlgaeSubsystem m_Algae = new AlgaeSubsystem();
    private final LimelightSubsystem m_Limelight = new LimelightSubsystem();

    // Subsystem initialization
    // swerve = new Swerve();
    // exampleSubsystem = new ExampleSubsystem();

    // Register Named Commands
    // NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
    // NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
    // NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

    // Do all other initialization
    

    // ...


    /* Path follower */
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        
        
        
        NamedCommands.registerCommand("Deposit L1", new CoralDepositRight(m_Coral, m_CANdle));
        
        NamedCommands.registerCommand("Coral Stop", new CoralStop(m_Coral));
        //NamedCommands.registerCommand("Test Forward", new InstantCommand(() -> m_Coral.testforward()));
        NamedCommands.registerCommand("DriveCmd", new DriveFail(drivetrain, 0.3, 3));
        //NamedCommands.registerCommand("Test Stop", new InstantCommand(() -> m_Coral.teststop()));
        NamedCommands.registerCommand("Elevator L1", new ElevatorL1(m_Elevator));
        NamedCommands.registerCommand("Elevator L3", new InstantCommand( () -> m_Elevator.thirdLevel()));
        NamedCommands.registerCommand("Elevator L4", new ElevatorL4(m_Elevator));
        NamedCommands.registerCommand("Deposit Any", new DepositForward(m_Coral));
        autoChooser = AutoBuilder.buildAutoChooser("6 Feet Test");
        SmartDashboard.putData( "Auto Mode", autoChooser);

        configureBindings();
    }


    private void configureBindings() {
        
        //Swerve Driving Code
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Preset Swerve Drive Buttons
        //   driveJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //   opJoystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(0, 0 ))));
        //   driveJoystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardstraight.withVelocityX(0.5).withVelocityY(0)) );
        //   driveJoystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardstraight.withVelocityX(-0.5).withVelocityY(0)) );

        // Run SysId routines when holding back/start and X/Y.
        //   Note that each routine should be run exactly once in a single log.
        //   driveJoystick.start().and(driveJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //   driveJoystick.start().and(driveJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        //   driveJoystick.back().and(driveJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //   driveJoystick.back().and(driveJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //   driveJoystick.leftBumper().whileTrue( new InstantCommand( () -> SignalLogger.start()));
        //   driveJoystick.rightBumper().whileTrue( new InstantCommand( () -> SignalLogger.stop()));

        // Reset the field-centric heading on left bumper press
        //   driveJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //   drivetrain.registerTelemetry(logger::telemeterize);
 
        //slower drive
        driveJoystick.rightTrigger().whileTrue( drivetrain.applyRequest(() ->
        drive.withVelocityX(-(driveJoystick.getLeftY() * MaxSpeed)/10) // Drive forward with negative Y (forward)
            .withVelocityY(-(driveJoystick.getLeftX() * MaxSpeed)/10) // Drive left with negative X (left)
            .withRotationalRate(-(driveJoystick.getRightX() * MaxAngularRate)/10) // Drive counterclockwise with negative X (left)
    ));
        
        opJoystick.a().whileTrue(new InstantCommand( () -> m_Elevator.switchFirst()));
        opJoystick.x().whileTrue(new InstantCommand( () -> m_Elevator.switchSecond()));                       //Button Masher prepares which elevator position 
        opJoystick.y().whileTrue(new InstantCommand( () -> m_Elevator.switchThird()));
        opJoystick.b().whileTrue(new InstantCommand( () -> m_Elevator.switchFourth()));
        opJoystick.rightTrigger().whileTrue(new CoralTakeTillFull(m_Coral, m_CANdle));
        //opJoystick.leftTrigger().whileTrue(new InstantCommand( () -> m_Coral.adjustCoral()));
        //opJoystick.leftBumper().whileTrue(new RunCoralCmd(m_Coral));

        // opJoystick.leftBumper().whileTrue(new InstantCommand( () -> m_Coral.switchToL1()));                   //Button Masher prepares sidesways launch for Level 1 
        // opJoystick.rightBumper().whileTrue(new InstantCommand( () -> m_Coral.switchOffL1()));                 //Button Masher prepares straight launch for other levels

        driveJoystick.a().whileTrue(new InstantCommand( () -> m_Elevator.firstLevel()));                      //Driver moves elevator to level 1
        driveJoystick.x().whileTrue(new InstantCommand( () -> m_Elevator.moveToPos()));                       //Driver moves elevator

        driveJoystick.y().whileTrue(new CoralDepositRight(m_Coral, m_CANdle)); 
        //driveJoystick.y().whileTrue(new InstantCommand( () -> m_Coral.testforward()));                            //Driver scores coral 
        driveJoystick.b().whileTrue(new InstantCommand( () -> m_Coral.stop()));                               //Driver emergency stops coral motors
        //driveJoystick.rightBumper().whileTrue( new InstantCommand( () -> m_Coral.slowerbackwards()));       //Temporary until its figured out in deposit
        
        driveJoystick.leftTrigger().whileTrue(new AlignToReefTagRelative(false, drivetrain)); // DOES NOT CURRENTLY DEPLOY
        //driveJoystick.leftBumper().whileTrue(new BTest(m_Coral ));
        
        
        // driveJoystick.rightTrigger().whileTrue(new InstantCommand( () -> m_Coral.getSensorValues()));
        // driveJoystick.leftBumper().whileTrue(new InstantCommand( () -> m_Coral.backwards()));


        // Manual Coral Button Code
        //driveJoystick.x().whileTrue(new InstantCommand( () -> m_Coral.slowerbackwards()));
        //driveJoystick.b().whileTrue(new InstantCommand( () -> m_Coral.forward()));
        //driveJoystick.pov(0).whileTrue(new InstantCommand( () -> m_Coral.depositRight()));
        // driveJoystick.rightTrigger().whileTrue(new InstantCommand( () -> m_Coral.slowercoral()));





        // Algae Code
        opJoystick.leftBumper().whileTrue(new AlgaeIntake(m_Algae));
        driveJoystick.leftBumper().whileTrue(new AlgaeDrop(m_Algae));
        opJoystick.pov(0).whileTrue(new InstantCommand( ()-> m_Algae.moveArmUpPosition()));
        
        opJoystick.pov(180).whileTrue(new InstantCommand( ()-> m_Algae.moveArmRestingPosition()));
        

        // opJoystick.pov(90).whileTrue( new InstantCommand( () -> m_Algae.manualmove()));
        // opJoystick.pov(180).whileTrue( new InstantCommand( () -> m_Algae.manualback()));
        opJoystick.pov(270).whileTrue(new InstantCommand( () -> m_Algae.manualstop()));
        //opJoystick.leftTrigger().whileTrue(new InstantCommand (() -> m_Algae.intakeAlgae()));

        

        // Coral Command Code
        // driveJoystick.pov(0).whileTrue(new CoralTakeTillFull(m_Coral));       // <--Just Shooter
        // driveJoystick.b().whileTrue(new CoralTakeTillFull(m_Coral, m_CANdle));   // <--Shooter and LED

    
    
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();

        // return new SequentialCommandGroup(
        //     new WaitCommand(.3),    
        //     // new DriveFail(drivetrain, 4, .85), // Turn 180 degrees
        //     new DriveFail(drivetrain, -4, 0.66),   //Turn 130 degrees
        //     new WaitCommand(1)
        // );
            

    }
}
