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
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
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

public class ElevatorWithPositionVoltage extends SubsystemBase {
    private static String RIO_CAN_LOOP_NAME = "rio";
    private static int TOP_ELEVATOR_CAN_ID = 16;
    private static int BOTTOM_ELEVATOR_CAN_ID = 17;

    private final CANBus bus = new CANBus(RIO_CAN_LOOP_NAME);
    private final TalonFX topMotor = new TalonFX(TOP_ELEVATOR_CAN_ID, bus);
    private final TalonFX bottomMotor = new TalonFX(BOTTOM_ELEVATOR_CAN_ID, bus);
    private final InvertedValue topMotorInverted = InvertedValue.Clockwise_Positive;

    private PositionVoltage positionVoltage = new PositionVoltage(0.0);

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

    // Motion profile constraints: max velocity (rot/s) and max acceleration
    // (rot/s^2)
    private final TrapezoidProfile rotationsProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(30, 50));
    // Target setpoint and current calculated profile state.
    private TrapezoidProfile.State rotationsSetpoint = new TrapezoidProfile.State(0, 0);

    // Publishers.
    //////////////
    private final DoublePublisher profiledSetPointMetersPublisher;
    private final DoublePublisher profiledSetPointVelocityMetersPerSecondPublisher;
    private final DoublePublisher currentPositionMetersPublisher;
    private final DoublePublisher voltagePublisher;

    public ElevatorWithPositionVoltage(NetworkTableInstance nt) {
        // Configure the motors.
        TalonFXConfiguration configs = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(topMotorInverted)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(Amps.of(17)))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(0.6)
                                .withKI(0)
                                .withKD(0)
                                .withKS(0.04)
                                .withKV(0.12)
                                .withKA(0.05)
                                .withKG(0.18));
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
        profiledSetPointVelocityMetersPerSecondPublisher = elevatorTable
                .getDoubleTopic("profiledSetPointVelocityMetersPerSecond").publish();
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

    // Implements an auto procedure lifting the elevator according to a motion
    // profile.
    //
    // Implements the block diagram shown in
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html#elevator-model-description
    //
    // Must be called from Robot.autonomousPeriodic().
    public void autonomousProfiledPeriodic() {
        double elapsedTime = autoTimer.get();
        double targetPositionMeters = 0.0;
        if (elapsedTime < 4.0) {
            targetPositionMeters = 0.4; // 40cm - Top
        } else if (elapsedTime < 6.0) {
            targetPositionMeters = 0.2; // 20cm - Midway
        } else if (elapsedTime < 8.0) {
            targetPositionMeters = 0.0; // 0cm - Bottom
            // Set target back at the bottom.
        } else if (elapsedTime < 10.0) {
            targetPositionMeters = 0.4; // 40cm - Top
        } else {
            targetPositionMeters = 0.0; // 0cm - Bottom
        }

        // 1. Calculate the next profile state step (dT is typically 0.02s for periodic)
        rotationsSetpoint = rotationsProfile.calculate(0.02, rotationsSetpoint,
                new TrapezoidProfile.State(targetPositionMeters / LINEAR_TRAVEL_METERS_PER_MOTOR_ROTATION, 0.0));

        topMotor.setControl(
                positionVoltage.withPosition(rotationsSetpoint.position).withVelocity(rotationsSetpoint.velocity));
    }

    @Override
    public void periodic() {
        motorVoltageSignal.refresh(); // per Javadoc, this initiates the signal refresh, but does not block for the
                                      // result, which means that here, we'd likely be still publishing the previous
                                      // value.

        profiledSetPointMetersPublisher.set(rotationsSetpoint.position * LINEAR_TRAVEL_METERS_PER_MOTOR_ROTATION);
        profiledSetPointVelocityMetersPerSecondPublisher
                .set(rotationsSetpoint.velocity * LINEAR_TRAVEL_METERS_PER_MOTOR_ROTATION);
        currentPositionMetersPublisher
                .set(topMotor.getPosition().getValue().in(Rotations) * LINEAR_TRAVEL_METERS_PER_MOTOR_ROTATION);
        voltagePublisher.set(motorVoltageSignal.getValue().in(Volts));
    }

    @Override
    public void simulationPeriodic() {
        // Feed the simulated battery voltage into the motor controller simulation state
        topMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage()); //

        // Fetch the motor's requested output voltage from the code
        double motorVoltage = topMotorSim.getMotorVoltage();

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
