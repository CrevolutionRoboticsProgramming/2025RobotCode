// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.crevolib.configs.CTREConfigs;
import frc.robot.driver.DriverXbox;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.drivetrain.TunerConstants;
import frc.robot.operator.OperatorXbox;

import static edu.wpi.first.units.Units.*;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  // private boolean constantRPM;

  /**
   * This method cancels all commands and returns subsystems to their default commands and the
   * gamepad configs are reset so that new bindings can be assigned based on mode This method
   * should be called when each mode is intialized
   */
  public static void resetCommandsAndButtons() {
    CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Reset Config for all gamepads and other button bindings
    // Driver.getInstance().resetConfig();
    // Operator.getInstance().resetConfig();
    DriverXbox.getInstance().resetConfig();
    OperatorXbox.getInstance().resetConfig();
  }


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    Logger.start();
    //CameraServer.startAutomaticCapture();
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
      resetCommandsAndButtons();
  }

  @Override
  public void disabledPeriodic() {
      
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // resetCommandsAndButtons();
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //     m_autonomousCommand.schedule();
    // }
    
  }

  /**
   * This function is called periodically during autonomous.
   */

  private final double kMaxVelocity = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double kMaxAngularVelocity = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(kMaxVelocity * 0.1)
    .withRotationalDeadband(kMaxAngularVelocity * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


  private Command runDrivetrain() {
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        new InstantCommand(() -> 
        CommandSwerveDrivetrain.getInstance().applyRequest(() -> 
        drive.withVelocityX(1.0 * kMaxVelocity)
        .withVelocityY(0.0)
        .withRotationalRate(0.0)
      ))
      )
    );
  }

  @Override
  public void autonomousPeriodic() {
      // CommandSwerveDrivetrain.getInstance().applyRequest(() -> 
      //   drive.withVelocityX(5.0 * kMaxVelocity)
      //   .withVelocityY(0.0)
      //   .withRotationalRate(0.0)
      // );
      
      // runDrivetrain();
  
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    resetCommandsAndButtons();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    resetCommandsAndButtons();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {
  }
}
