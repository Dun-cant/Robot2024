// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /* Subsystems and Commands */
    private final Swerve swerve = new Swerve();

    /* Drive Controls */
    private final SendableChooser<Command> autonChooser;
    private boolean slowEnabled;

    /* Controllers */
    private final CommandXboxController xbox = new CommandXboxController(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Settings
        slowEnabled = false;

        // Configure the trigger bindings
        configureBindings();

        // Autos
        autonChooser = new SendableChooser<>();
        autonChooser.addOption("nothing", new InstantCommand());
        Shuffleboard.getTab("Auto").add(autonChooser);
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
        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve, 
                        () -> (xbox.getLeftY() * (slowEnabled ? 0.5 : 1)) * 0.8,
                        () -> (xbox.getLeftX() * (slowEnabled ? 0.5 : 1)) * 0.8, 
                        () -> (xbox.getRightX() * 0.75 * (slowEnabled ? 0.5 : 1)) * 0.6
                )
        );

        xbox.b().onTrue(new InstantCommand(swerve::toggleBrake));
        xbox.y().onTrue(new InstantCommand(swerve::zeroGyro));
        xbox.x().onTrue(new InstantCommand(() -> slowEnabled = !slowEnabled));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
