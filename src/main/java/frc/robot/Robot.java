// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the 
// WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in the TimedRobot documentation. If you change the name of
 * this class or the package after creating this project, you must also update
 * the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    // Option 1: Simple elevator implementation with WPILib Motion Profile.
    // private final Elevator elevator = new
    // Elevator(NetworkTableInstance.getDefault());

    // Option 2: Elevator implementation with Motion Magic.
    private final ElevatorWithMotionMagic elevatorWithMotionMagic = new ElevatorWithMotionMagic(
            NetworkTableInstance.getDefault());
    private final CommandXboxController controller = new CommandXboxController(0);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    public Robot() {
        elevatorWithMotionMagic.setDefaultCommand(
                elevatorWithMotionMagic.runManual(() -> MathUtil.applyDeadband(-controller.getLeftY(), 0.1)));
    }

    @Override
    public void robotPeriodic() {

        // elevator.periodic();

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        // elevator.autonomousInit();
        elevatorWithMotionMagic.autonomousInit();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // This is the method where the elevator is controlled by a motion profile.
        // It performs a pre-recorded sequence of movements.
        // elevator.autonomousProfiledPeriodic();

        // This is a simpler method where the elevator is controlled by a simple voltage
        // routine.
        // elevator.autonomousVoltagePeriodic();

        // This is the method where the elevator is controlled by a motion magic
        // routine.
        elevatorWithMotionMagic.autonomousProfiledPeriodic();
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        // elevator.simulationPeriodic();
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }
}
