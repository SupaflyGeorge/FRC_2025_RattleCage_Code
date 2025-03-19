// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
//import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import static edu.wpi.first.units.Units.NewtonMeters;

import java.io.File;
import java.lang.ProcessBuilder.Redirect.Type;
import java.time.format.SignStyle;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import swervelib.SwerveInputStream;
import frc.robot.commands.ClimbSet;
import frc.robot.commands.ElevatorDown;

import frc.robot.commands.ElevatorUp;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.OutTakeCoral;
import frc.robot.commands.ElevatorSet;
//import frc.robot.commands.ClimbSet;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.PivotSet;



import frc.robot.subsystems.mechanisms.Elevator;
import frc.robot.subsystems.mechanisms.AlgaeIntake;
import frc.robot.subsystems.mechanisms.Climb;
import frc.robot.subsystems.mechanisms.Pivot;
import frc.robot.subsystems.mechanisms.Intake;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  //Pathplanning Commands are defined here...
  public SendableChooser<Command> autoChooser;

  //Robots Subsystems and Commands are defined here
  public static final Intake intake = new Intake();
  public static final Elevator elevator = new Elevator();
  public static final Pivot pivot = new Pivot();
  public static final Climb climb = new Climb();
  public static final AlgaeIntake algaeIntake = new AlgaeIntake();
  
 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);


  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(1)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
        configureShuffleboard();
      // Configure the trigger bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    
      
    
        //Refer to this when needed -> NamedCommands.registerCommand("Command", new Command(subsystem, subsystem));
        //Named Commands to run for Autonomous Routes--------------------------------------------
    
  
        
        NamedCommands.registerCommand("ElevatorL4Up", new ElevatorUp(elevator, 0.5));
        NamedCommands.registerCommand("ElevatorL4UpStop", new ElevatorUp(elevator, 0));
        NamedCommands.registerCommand("ElevatorL4Down", new ElevatorDown(elevator, 0.5));
        NamedCommands.registerCommand("ElevatorL4DownStop", new ElevatorDown(elevator, 0));
        NamedCommands.registerCommand("Score Coral", new IntakeCoral(intake, 0.5));
        NamedCommands.registerCommand("Score Coral Stop", new IntakeCoral(intake, 0));
        NamedCommands.registerCommand("Intake Algae", new IntakeAlgae(algaeIntake, 0.5));
        NamedCommands.registerCommand("Intake Algae Stop", new IntakeAlgae(algaeIntake, 0));
        NamedCommands.registerCommand("Extake Algae", new IntakeAlgae(algaeIntake, -0.5));
        NamedCommands.registerCommand("Extake Algae Stop", new IntakeAlgae(algaeIntake, 0));
    
    
        //Default Auto that auto selects once the robot turns on--------------------------------------------------------
        autoChooser = AutoBuilder.buildAutoChooser("None");
        //Calls the "Field" tab and adds list of Autonomous Routines----------------------------------------------------
        Shuffleboard.getTab("Field").add("Autonomous Routes", autoChooser);
    
        
    
      }
    
      private void configureShuffleboard() {
    CameraServer.startAutomaticCapture(0);
    CameraServer.putVideo("Bingus", 200, 100);
 }

      /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    //Autonomous Routines

   // autoChooser.addOption("MiddleCoralL4", new PathPlannerAuto("MiddleCoralL4"));



/*    
    //Lower Position Autons------------------------------------------------------------------------------------------
    autoChooser.addOption("LowerPos-LowerAlgae", new PathPlannerAuto("LowerPos-LowerAlgae"));
    autoChooser.addOption("LowerPos-MiddleAlgae", new PathPlannerAuto("LowerPos-MiddleAlgae"));
    autoChooser.addOption("LowerPos-UpperAlgae", new PathPlannerAuto("LowerPos-UpperAlgae"));
    autoChooser.addOption("LowerPosCLose-CoralLevel4", new PathPlannerAuto("LowerPosCLose-CoralLevel4"));
    autoChooser.addOption("LowerPosFar-CoralLevel4", new PathPlannerAuto("LowerPosFar-CoralLevel4"));
*/
    //Middle Position Autons------------------------------------------------------------------------------------------
   /*  autoChooser.addOption("MiddlePos-LowerAlgae", new PathPlannerAuto("MiddlePos-LowerAlgae"));
    autoChooser.addOption("MiddlePos-MiddleAlgae", new PathPlannerAuto("MiddlePos-MiddleAlgae"));
    autoChooser.addOption("MiddlePos-UpperAlgae", new PathPlannerAuto("MiddlePos-UpperAlgae"));
    autoChooser.addOption("MiddlePos-CoralLevel4", new PathPlannerAuto("MiddlePos-CoralLevel4"));*/
/*
    //Upper Position Autons------------------------------------------------------------------------------------------
    autoChooser.addOption("UpperPos-LowerAlgae", new PathPlannerAuto("UpperPos-LowerAlgae"));
    autoChooser.addOption("UpperPos-MiddleAlgae", new PathPlannerAuto("UpperPos-MiddleAlgae"));
    autoChooser.addOption("UpperPos-UpperAlgae", new PathPlannerAuto("UpperPos-UpperAlgae"));
    autoChooser.addOption("UpperPosCLose-CoralLevel4", new PathPlannerAuto("UpperPosCLose-CoralLevel4"));
    autoChooser.addOption("UpperPosFar-CoralLevel4", new PathPlannerAuto("UpperPosFar-CoralLevel4"));
*/
    //Driver Controllers

    //B -> Intakes the Coral-------------------------------------------------------
    operatorXbox.b().onTrue(new IntakeCoral(intake, .5));
    operatorXbox.b().onFalse(new IntakeCoral(intake, 0));
    //A -> OutTakes the Coral------------------------------------------------------
    operatorXbox.a().onTrue(new IntakeCoral(intake, -.5));
    operatorXbox.a().onFalse(new IntakeCoral(intake, 0));
    //Right Bumper -> Elevator goes up---------------------------------------------
    operatorXbox.rightBumper().onTrue(new ElevatorUp(elevator, .75));
    operatorXbox.rightBumper().onFalse(new ElevatorUp(elevator, 0));
    //Left Bumper -> Elevator goes down--------------------------------------------
    operatorXbox.leftBumper().onTrue(new ElevatorDown(elevator, .75));
    operatorXbox.leftBumper().onFalse(new ElevatorDown(elevator, 0));
    //Left Bumper -> Climb goes up-------------------------------------------------
    operatorXbox.rightTrigger().onTrue(new IntakeAlgae(algaeIntake, -1.0));
    operatorXbox.rightTrigger().onFalse(new IntakeAlgae(algaeIntake, 0));
    //Left Trigger -> Intakes the Algae--------------------------------------------
    operatorXbox.leftTrigger().onTrue(new IntakeAlgae(algaeIntake, 1.0));
    operatorXbox.leftTrigger().onFalse(new IntakeAlgae(algaeIntake,0));
    //X -> Pivot goes up-----------------------------------------------------------
    operatorXbox.povUp().onTrue(new PivotSet(pivot, 0.5));
    operatorXbox.povUp().onFalse(new PivotSet(pivot, 0));
    //Y -> Pivot goes down---------------------------------------------------------
    operatorXbox.povDown().onTrue(new PivotSet(pivot, -0.5));
    operatorXbox.povDown().onFalse(new PivotSet(pivot, 0));


    Command pivotZero = new PivotSet(pivot, (0.8))
     .until (() -> pivot.getPosition() < 1);
     operatorXbox.povLeft().onTrue(pivotZero);
    //Left Bumper -> Climb goes up-------------------------------------------------
    //driverXbox.rightBumper().onTrue(new ClimbSet(climb, 0.5));
    //driverXbox.rightBumper().onFalse(new ClimbSet(climb, 0));
    //Left Bumper -> Climb goes down-----------------------------------------------
    //driverXbox.leftBumper().onTrue(new ClimbSet(climb, -0.5));
    //driverXbox.leftBumper().onFalse(new ClimbSet(climb, 0));
    // Elevator Setpoints (ONLY GEORGE EDIT)---------------------------------------
    Command level1 = new ElevatorSet(elevator, () -> .5)
    .until (() -> elevator.getPosition() < 0);
    driverXbox.y().onTrue(level1);

    Command level2 = new ElevatorSet(elevator, () -> -.5)
    .until (() -> elevator.getPosition() > 40);
    driverXbox.b().onTrue(level2);

    Command level3 = new ElevatorSet(elevator, () -> -.5)
    .until (() -> elevator.getPosition() > 75);
    driverXbox.x().onTrue(level3);

    Command level4 = new ElevatorSet(elevator, () -> -.5)
    .until (() -> elevator.getPosition() > 120);
    driverXbox.a().onTrue(level4);

    driverXbox.rightBumper().onTrue(new ElevatorUp(elevator, .75));
    driverXbox.rightBumper().onFalse(new ElevatorUp(elevator, 0));
    //Left Bumper -> Elevator goes down--------------------------------------------
    driverXbox.leftBumper().onTrue(new ElevatorDown(elevator, .75));
    driverXbox.leftBumper().onFalse(new ElevatorDown(elevator, 0));

     //B -> Intakes the Coral-------------------------------------------------------
     driverXbox.povUp().onTrue(new IntakeCoral(intake, .5));
     driverXbox.povUp().onFalse(new IntakeCoral(intake, 0));
     //A -> OutTakes the Coral------------------------------------------------------
     driverXbox.povDown().onTrue(new IntakeCoral(intake, -.5));
     driverXbox.povDown().onFalse(new IntakeCoral(intake, 0));

    //driverXbox.rightTrigger().onTrue(new IntakeAlgae(algaeIntake, -1.0));
    //driverXbox.rightTrigger().onFalse(new IntakeAlgae(algaeIntake, 0));
    //Left Trigger -> Intakes the Algae--------------------------------------------
    //driverXbox.leftTrigger().onTrue(new IntakeAlgae(algaeIntake, 1.0));
    //driverXbox.leftTrigger().onFalse(new IntakeAlgae(algaeIntake,0));

    //driverXbox.povUp().onTrue(new ClimbSet(climb, 0.5));
    //driverXbox.povUp().onFalse(new ClimbSet(climb, 0));
    //driverXbox.povDown().onTrue(new ClimbSet(climb, -1));
    //driverXbox.povDown().onFalse(new ClimbSet(climb, 0));

     //X -> Pivot goes up-----------------------------------------------------------
     //driverXbox.povUp().onTrue(new PivotSet(pivot, -0.5));
     //driverXbox.povUp().onFalse(new PivotSet(pivot, 0));
     //Y -> Pivot goes down---------------------------------------------------------
     //driverXbox.povDown().onTrue(new PivotSet(pivot, 0.5));
     //driverXbox.povDown().onFalse(new PivotSet(pivot, 0));

     //Command pivotZero = new PivotSet(pivot, (0.8)) .until (() -> pivot.getPosition() < 1);
     //driverXbox.povLeft().onTrue(pivotZero);
    //Right Joystick Button (R3) -> Zero's the Gyro from the front of the Robot--
    //driverXbox.rightStick().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    

    //Base Commands from YAGSL---------------------------------------------------    
    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    
          
          
            //------------------------------------------------------------------------------------------------------------------
          
          
              Command driveFieldOrientedDirectAngle         = drivebase.driveFieldOriented(driveDirectAngle);
              Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
              Command driveSetpointGen                      = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
              Command driveFieldOrientedDirectAngleSim      = drivebase.driveFieldOriented(driveDirectAngleSim);
              Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
              Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(
                  driveDirectAngleSim);
          
              if (RobotBase.isSimulation())
              {
                drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
              } else
              {
                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
              }
          
              {
                driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
              }
          
            }

            /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  //Calls the autoChooser once Autonomous runs
    return autoChooser.getSelected();

  //return drivebase.getAutonomousCommand("New Auto");
  //return new InstantCommand();
  }

//------------------------------------------------------------------------------------------------------------------
//Temporary Placement of Commands for Named Commands
//Make Named Commands into Java files in the commands section once mechanisms are ready and available
 /*  public Command AdjustForProcessor() {
      return null;
  }

  public Command PickUp_Algae() {
    return null;
  }

  public Command AdjustForCoral() {
    return null;
}
*/
//------------------------------------------------------------------------------------------------------------------

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

}