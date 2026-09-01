// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.currentMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControlScheme;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.BetterAutoChooser;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.RobotUtil;
import frc.robot.util.io.GuitarHeroController;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // subsystems
  private final Drive drive;
  private final Vision vision;

  // controllers
  private ControlScheme controlScheme = ControlScheme.MAIN;
  private final CommandXboxController driverController =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);
  private GuitarHeroController guitarHeroController;

  // default drive commands
  private Command defaultDriveCommand;
  private Command guitarHeroDriveCommand;

  // dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Simulated things
  private SwerveDriveSimulation driveSimulation;
  //  private SuperstructureSim superstructureSim;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (currentMode) {
      case REAL -> {
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft, false),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight, false),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft, false),
                new ModuleIOTalonFXReal(TunerConstants.BackRight, false),
                (pose) -> {});
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_0_NAME, VisionConstants.CAMERA_0_OFFSET),
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_1_NAME, VisionConstants.CAMERA_1_OFFSET),
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_2_NAME, VisionConstants.CAMERA_2_OFFSET));
      }
      case SIM -> {
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()) {},
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_0_NAME,
                    VisionConstants.CAMERA_0_OFFSET,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_1_NAME,
                    VisionConstants.CAMERA_1_OFFSET,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_2_NAME,
                    VisionConstants.CAMERA_2_OFFSET,
                    driveSimulation::getSimulatedDriveTrainPose));
      }
      default -> {
        /* REPLAY */
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision =
            new Vision(
                drive, new VisionIO() {}, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
      }
    }

    PhoenixUtil.startTelemetry();

    // Configure the trigger bindings
    configureBindings();

    LoggedDashboardChooser<ControlScheme> controlProfiles =
        new LoggedDashboardChooser<>("Control Profile");
    controlProfiles.addDefaultOption("Main", ControlScheme.MAIN);
    controlProfiles.addOption("Guitar Hero Operator", ControlScheme.GUITAR_HERO_OP);
    controlProfiles.addOption("Guitar Hero Full Control", ControlScheme.GUITAR_HERO_FULL);
    controlProfiles.addOption("Testing", ControlScheme.TEST);
    controlProfiles.onChange(this::setControlScheme);

    // Set up commands for PathPlanner
    configureAutoCommands();

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser =
        new LoggedDashboardChooser<>("Auto Chooser", BetterAutoChooser.buildAutoChooser());

    // Set up SysId routines
    //    autoChooser.addOption(
    //        "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    //    autoChooser.addOption(
    //        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    //    autoChooser.addOption(
    //        "Drive SysId (Quasistatic Forward)",
    //        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //    autoChooser.addOption(
    //        "Drive SysId (Quasistatic Reverse)",
    //        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    //    autoChooser.addOption(
    //        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //    autoChooser.addOption(
    //        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Set up custom autos (non-PathPlanner)
    //    autoChooser.addOption("Full System Check", Autos.systemCheck(drive, shooter, feeder,
    // intake));
    //    autoChooser.addOption(
    //        "Dynamic Left Cycle", Autos.leftCycle(drive, vision, shooter, feeder, intake));
    //    autoChooser.addOption(
    //        "Dynamic Right Cycle", Autos.rightCycle(drive, vision, shooter, feeder, intake));

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    RobotUtil.setDriverController(driverController);
    RobotUtil.setOperatorController(operatorController);

    // Lock wheels to X pattern
    Command lockWheels = Commands.startEnd(drive::stopWithX, () -> {}, drive);
    // Reset gyro to 0°
    Command zeroGyro = Commands.runOnce(() -> drive.zeroGyro(true), drive).ignoringDisable(true);

    // Default command, normal field-relative drive
    useDefaultDrive();

    if (currentMode == Constants.Mode.SIM) {
      CommandGenericHID keyboard = new CommandGenericHID(3);
    }

    if (DriverStation.isTest()) {
      // single controller for testing
      driverController.x().whileTrue(lockWheels);
      driverController.povLeft().onTrue(zeroGyro);

      ;
    } else {
      /* driver controls */
      driverController.x().whileTrue(lockWheels);
      driverController.povLeft().onTrue(zeroGyro);

      /* operator controls */

      // test mode (single controller)

      // main profile

    }
  }

  public void setControlScheme(ControlScheme newScheme) {
    switch (newScheme) {
      case MAIN:
      case TEST:
        if (controlScheme == ControlScheme.GUITAR_HERO_FULL) {
          useDefaultDrive();
        }
        break;
      case GUITAR_HERO_OP:
        configureGuitarHeroController(false);
        break;
      case GUITAR_HERO_FULL:
        configureGuitarHeroController(true);
        break;
      default:
    }
    controlScheme = newScheme;
  }

  private void configureGuitarHeroController(boolean fullControl) {
    if (guitarHeroController == null) {
      // lazy instantiation
      guitarHeroController =
          new GuitarHeroController(ControllerConstants.GUITAR_HERO_CONTROLLER_PORT);

      // configure triggers only once

      // controls are only active during the correct mode
      BooleanSupplier guitarHeroControls = () -> controlScheme.isGuitarHero;
      BooleanSupplier guitarHeroDrive = () -> controlScheme == ControlScheme.GUITAR_HERO_FULL;
    }

    if (fullControl) {
      useGuitarHeroDrive();
    } else {
      useDefaultDrive();
    }
  }

  private void configureAutoCommands() {}

  private void useDefaultDrive() {
    if (defaultDriveCommand == null) {
      defaultDriveCommand =
          DriveCommands.joystickDrive(
              drive,
              () -> -driverController.getLeftY(),
              () -> -driverController.getLeftX(),
              () -> -driverController.getRightX());
    }
    drive.setDefaultCommand(defaultDriveCommand);
    Command currentDriveCommand = drive.getCurrentCommand();
    if (currentDriveCommand != null) currentDriveCommand.cancel();
  }

  private void useGuitarHeroDrive() {
    if (guitarHeroDriveCommand == null) {
      guitarHeroDriveCommand =
          DriveCommands.joystickDrive(
              drive,
              () -> -guitarHeroController.getJoystickY(),
              () -> -guitarHeroController.getJoystickX(),
              () -> -guitarHeroController.getStrumBarAxis());
    }
    drive.setDefaultCommand(guitarHeroDriveCommand);
    Command currentDriveCommand = drive.getCurrentCommand();
    if (currentDriveCommand != null) currentDriveCommand.cancel();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {}

  public void updateSimulation() {}
}
