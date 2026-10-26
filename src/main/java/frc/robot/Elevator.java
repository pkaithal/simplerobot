package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static String RIO_CAN_LOOP_NAME = "rio";
    private static int TOP_ELEVATOR_CAN_ID = 16;
    private static int BOTTOM_ELEVATOR_CAN_ID = 17;

    private final CANBus bus = new CANBus(RIO_CAN_LOOP_NAME);
    private final TalonFX topMotor = new TalonFX(TOP_ELEVATOR_CAN_ID, bus);
    private final TalonFX bottomMotor = new TalonFX(BOTTOM_ELEVATOR_CAN_ID, bus);
    private final InvertedValue topMotorInverted = InvertedValue.Clockwise_Positive;

    private VoltageOut elevatorVoltage = new VoltageOut(0.0);

    private StatusSignal<Voltage> motorVoltageSignal;

    // Simulation-only definitions
    private final TalonFXSimState topMotorSim = topMotor.getSimState();
    private final ElevatorSim elevatorSim;

    // Constants for the mechanism simulation parameters
    // 70:12 gearbox; corresponds to motor-rotations/sprocket-rotations conversion
    private static final double GEAR_REDUCTION = 70.0 / 12.0;
    private static final Mass CARRIAGE_MASS = Kilograms.of(5.0); // 5 kg elevator carriage
    private static final Distance SPROCKET_PITCH_RADIUS = Inches.of(1.751 / 2.0); // 1" radius pulley
    private static final Distance MIN_ELEVATOR_HEIGHT = Meters.of(0.0);
    private static final Distance MAX_ELEVATOR_HEIGHT = Meters.of(0.5);
    private static final double LINEAR_TRAVEL_METERS_PER_MOTOR_ROTATION = 2 * Math.PI * SPROCKET_PITCH_RADIUS.in(Meters)
            / GEAR_REDUCTION;

    // Voltage based routine variables.
    ///////////////////////////////////

    // Voltage based routine timer. Used to determine when to change from one
    // phase of the routine to another.
    private final Timer autoTimer = new Timer();

    // Motion profile based routine variables.
    //////////////////////////////////////////

    // Motion profile constraints: max velocity (m/s) and max acceleration (m/s^2)
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(3, 3);
    // WPILib Motion Profiler
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    // Target setpoint and current calculated profile state.
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0, 0);

    // Feedback and feedforward controllers driving the elevator, following the
    // motion profile setpoints.
    private final PIDController pid = new PIDController(10, 0.0, 2);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.05, 0.28, 4.8, 4.0);
    // Values Stefan found with System Identification routine.
    // ks = 0.138
    // kg = 0.20163

    // Publishers.
    //////////////
    private final DoublePublisher profiledSetPointMetersPublisher;
    private final DoublePublisher currentPositionMetersPublisher;
    private final DoublePublisher voltagePublisher;

    public Elevator(NetworkTableInstance nt) {
        // Configure the motors.
        TalonFXConfiguration configs = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(topMotorInverted)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(Amps.of(17)));
        topMotor.getConfigurator().apply(configs);
        bottomMotor.getConfigurator().apply(configs);

        bottomMotor.setControl(
                new Follower(TOP_ELEVATOR_CAN_ID, MotorAlignmentValue.Aligned));

        // 2. Initialize the WPILib elevator physics simulation plant
        elevatorSim = new ElevatorSim(
                DCMotor.getKrakenX60(2), // Modeling 2 motors total on the gearbox
                GEAR_REDUCTION,
                CARRIAGE_MASS.in(Kilograms),
                SPROCKET_PITCH_RADIUS.in(Meters),
                MIN_ELEVATOR_HEIGHT.in(Meters),
                MAX_ELEVATOR_HEIGHT.in(Meters),
                true, // Simulate gravity
                0.0 // Starting height
        );
        topMotorSim.Orientation = topMotorInverted == InvertedValue.Clockwise_Positive
                ? ChassisReference.Clockwise_Positive
                : ChassisReference.CounterClockwise_Positive;

        motorVoltageSignal = topMotor.getMotorVoltage();

        var elevatorTable = nt.getTable("Robot/Elevator");
        profiledSetPointMetersPublisher = elevatorTable.getDoubleTopic("profiledSetPointMeters").publish();
        currentPositionMetersPublisher = elevatorTable.getDoubleTopic("currentPositionMeters").publish();
        voltagePublisher = elevatorTable.getDoubleTopic("voltage").publish();
    }

    // Call from Robot.autonomousInit() to initialize auto states.
    public void autonomousInit() {
        // Resets timer to 0 and starts running
        autoTimer.reset();
        autoTimer.start();

        // Reset the encoder position to 0 at the start of autonomous
        topMotor.setPosition(0.0);
    }

    // Helper function to compute elevator vertical position from
    // the rotation position of the motors.
    private Distance curPosition() {
        // How to compute the elevator vertical position from the rotation position of
        // the motors:
        // https://docs.google.com/presentation/d/1rl5EaBWSbhddaBKi7KndIaIQq9RJXTfchpoNX1QDWg0/edit?slide=id.p#slide=id.p
        // return Inches.of(topMotor.getPosition().getValueAsDouble() * 0.94);
        return Meters.of(topMotor.getPosition().getValue().in(Rotations) * LINEAR_TRAVEL_METERS_PER_MOTOR_ROTATION);
    }

    // Implements a simple auto procedure with three fixed voltage/duration stages
    // Must be called from Robot.autonomousPeriodic().
    public void autonomousVoltagePeriodic() {
        double elapsedTime = autoTimer.get();
        Voltage voltageOutput;
        if (elapsedTime < 2.0) {
            // Run motors at 50% for 2 seconds
            voltageOutput = Volts.of(1.0);
        } else if (elapsedTime < 6.0) {
            // Stop motors after 2 seconds
            voltageOutput = Volts.of(-0.4);
        } else {
            // Stop motors after that
            voltageOutput = Volts.of(0.0);
        }

        topMotor.setControl(
                elevatorVoltage.withOutput(voltageOutput));
    }

    // Implements an auto procedure lifting the elevator according to a motion
    // profile.
    //
    // Implements the block diagram shown in
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html#elevator-model-description
    //
    // Must be called from Robot.autonomousPeriodic().
    public void autonomousProfiledPeriodic() {
        double elapsedTime = autoTimer.get();
        double targetPosition = 0.0;
        if (elapsedTime < 4.0) {
            targetPosition = 0.4; // 40cm - Top
        } else if (elapsedTime < 6.0) {
            targetPosition = 0.2; // 20cm - Midway
        } else if (elapsedTime < 8.0) {
            targetPosition = 0.0; // 0cm - Bottom
            // Set target back at the bottom.
        } else if (elapsedTime < 10.0) {
            targetPosition = 0.4; // 40cm - Top
        } else {
            targetPosition = 0.0; // 0cm - Bottom
        }
        // 1. Calculate the next profile state step (dT is typically 0.02s for periodic)
        setpoint = profile.calculate(0.02, setpoint, new TrapezoidProfile.State(targetPosition, 0.0));

        // 2. Get current position reading from the motor (in rotations)
        // Since for this mechanism, one rotation ~ 1 inch, we can use these as inches.
        double currentPosition = curPosition().in(Meters);

        // 3. Calculate PID output based on setpoint position vs current position
        double pidOutputVolts = pid.calculate(currentPosition, setpoint.position);

        // 4. Calculate Feedforward voltage using position setpoint velocity &
        // acceleration
        double ffOutputVolts = feedforward.calculate(setpoint.velocity);

        // 5. Apply total output voltage to the TalonFX via VoltageOut
        double totalVoltage = pidOutputVolts + ffOutputVolts;
        topMotor.setControl(elevatorVoltage.withOutput(totalVoltage));
    }

    @Override
    public void periodic() {
        motorVoltageSignal.refresh(); // per Javadoc, this initiates the signal refresh, but does not block for the
                                      // result, which means that here, we'd likely be still publishing the previous
                                      // value.

        profiledSetPointMetersPublisher.set(setpoint.position);
        currentPositionMetersPublisher
                .set(topMotor.getPosition().getValue().in(Rotations) * LINEAR_TRAVEL_METERS_PER_MOTOR_ROTATION);
        voltagePublisher.set(motorVoltageSignal.getValue().in(Volts));
    }

    @Override
    public void simulationPeriodic() {
        // Feed the simulated battery voltage into the motor controller simulation state
        topMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage()); //

        // Fetch the motor's requested output voltage from the code
        double motorVoltage = topMotorSim.getMotorVoltage(); //

        // Pass the voltage into the physical simulation environment
        elevatorSim.setInput(motorVoltage);

        // Step the simulation forward by 20ms (standard robot loop)
        elevatorSim.update(0.020);

        // Convert the linear height back to motor rotations to update internal sensor
        // data
        double linearPosition = elevatorSim.getPositionMeters();
        double linearVelocity = elevatorSim.getVelocityMetersPerSecond();

        // Force the simulated motor's sensor to read our simulated physical mechanism
        // values
        topMotorSim.setRawRotorPosition(linearPosition / LINEAR_TRAVEL_METERS_PER_MOTOR_ROTATION);
        topMotorSim.setRotorVelocity(linearVelocity / LINEAR_TRAVEL_METERS_PER_MOTOR_ROTATION);
    }
}
