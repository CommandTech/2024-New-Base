// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.List;
import java.util.Map.Entry;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private CANSparkMax topFlywheelMotor;
    private CANSparkMax bottomFlywheelMotor;
    private CANSparkMax angleMotor;
    private CANSparkMax indexerMotor;

    public Hardware(CANSparkMax lSlaveMotor,
                    CANSparkMax rMasterMotor,
                    CANSparkMax angleMotor,
                    CANSparkMax indexerMotor) {
      this.topFlywheelMotor = lSlaveMotor;
      this.bottomFlywheelMotor = rMasterMotor;
      this.angleMotor = angleMotor;
      this.indexerMotor = indexerMotor;
    }
  }

  /** Shooter state */
  public static class State {
    public final Measure<Velocity<Distance>> speed;
    public final Measure<Angle> angle;

    public static final State AMP_PREP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(55.0));
    public static final State AMP_SCORE_STATE = new State(Units.MetersPerSecond.of(+2.5), Units.Degrees.of(55.0));
    public static final State SPEAKER_PREP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(54.0));
    public static final State SPEAKER_SCORE_STATE = new State(Units.MetersPerSecond.of(+15.0), Units.Degrees.of(56.0));
    public static final State SOURCE_PREP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(55.0));
    public static final State SOURCE_INTAKE_STATE = new State(Units.MetersPerSecond.of(-10.0), Units.Degrees.of(55.0));

    public State(Measure<Velocity<Distance>> speed, Measure<Angle> angle) {
      this.speed = speed;
      this.angle = angle;
    }
  }

  private static final Measure<Velocity<Distance>> ZERO_FLYWHEEL_SPEED = Units.MetersPerSecond.of(0.0);
  private static final int FLYWHEEL_CURRENT_LIMIT = 40;
  private static final int ANGLE_MOTOR_CURRENT_LIMIT = 20;
  // TODO: Figure out current limit for shooter angle
  private static final Measure<Dimensionless> INDEXER_SPEED = Units.Percent.of(100.0);
  private static final Measure<Dimensionless> INDEXER_SLOW_SPEED = Units.Percent.of(4.0);

  private final Measure<Distance> MIN_SHOOTING_DISTANCE = Units.Meters.of(0.0);
  private final Measure<Distance> MAX_SHOOTING_DISTANCE;
  private final Measure<Velocity<Distance>> MAX_FLYWHEEL_SPEED;

  private CANSparkMax m_topFlywheelMotor;
  private CANSparkMax m_bottomFlywheelMotor;
  private CANSparkMax m_angleMotor;
  private CANSparkMax m_indexerMotor;

  private TrapezoidProfile.Constraints m_angleConstraint;
  private Supplier<Pose2d> m_poseSupplier;
  private Supplier<Pair<Integer,Translation2d>> m_targetSupplier;

  private SparkPIDController m_flywheelConfig;
  private SparkPIDController m_angleConfig;

  private State m_desiredShooterState;

  private Mechanism2d m_mechanism2d;

  /**
   * Create an instance of ShooterSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param shooterHardware Hardware devices required by shooter
   * @param flywheelConfig Flywheel PID config
   * @param angleConfig Angle adjust PID config
   * @param angleFF Angle adjust feed forward gains
   * @param angleConstraint Angle adjust motion constraint
   * @param flywheelDiameter Flywheel diameter
   * @param shooterMap Shooter lookup table
   * @param poseSupplier Robot pose supplier
   * @param targetSupplier Speaker target supplier
   */
  public ShooterSubsystem(Hardware shooterHardware, SparkPIDController flywheelConfig, SparkPIDController angleConfig,
                          TrapezoidProfile.Constraints angleConstraint, Measure<Distance> flywheelDiameter,
                          List<Entry<Measure<Distance>, State>> shooterMap,
                          Supplier<Pose2d> poseSupplier, Supplier<Pair<Integer,Translation2d>> targetSupplier) {
    setSubsystem(getClass().getSimpleName());
    // MAX_FLYWHEEL_SPEED = Units.MetersPerSecond.of((/* Get the max speed */) * (flywheelDiameter.in(Units.Meters) * Math.PI));
    MAX_FLYWHEEL_SPEED = Units.MetersPerSecond.of(50.0);
    this.m_topFlywheelMotor = shooterHardware.topFlywheelMotor;
    this.m_bottomFlywheelMotor = shooterHardware.bottomFlywheelMotor;
    this.m_angleMotor = shooterHardware.angleMotor;
    this.m_indexerMotor = shooterHardware.indexerMotor;
    this.m_flywheelConfig = flywheelConfig;
    this.m_angleConfig = angleConfig;
    this.m_angleConstraint = angleConstraint;
    this.m_poseSupplier = poseSupplier;
    this.m_targetSupplier = targetSupplier;

    // Slave bottom flywheel motor to top
    m_bottomFlywheelMotor.follow(m_topFlywheelMotor);

    // Initialize PID
    // m_topFlywheelMotor.initializeSparkPID(m_flywheelConfig, FeedbackSensor.NEO_ENCODER);
    // m_angleMotor.initializeSparkPID(m_angleConfig, FeedbackSensor.THROUGH_BORE_ENCODER, true, true);

    // Set flywheel conversion factor
    // var flywheelConversionFactor = flywheelDiameter.in(Units.Meters) * Math.PI;
    // m_topFlywheelMotor.setPositionConversionFactor(FeedbackSensor.NEO_ENCODER, flywheelConversionFactor);
    // m_topFlywheelMotor.setVelocityConversionFactor(FeedbackSensor.NEO_ENCODER, flywheelConversionFactor / 60);

    // Set angle adjust conversion factor
    // var angleConversionFactor = Math.PI * 2;
    // m_angleMotor.setPositionConversionFactor(FeedbackSensor.THROUGH_BORE_ENCODER, angleConversionFactor);
    // m_angleMotor.setVelocityConversionFactor(FeedbackSensor.THROUGH_BORE_ENCODER, angleConversionFactor / 60);

    // Set idle mode
    m_topFlywheelMotor.setIdleMode(IdleMode.kCoast);
    m_bottomFlywheelMotor.setIdleMode(IdleMode.kCoast);
    m_angleMotor.setIdleMode(IdleMode.kBrake);
    m_indexerMotor.setIdleMode(IdleMode.kBrake);

    // Set current limits
    m_topFlywheelMotor.setSmartCurrentLimit(FLYWHEEL_CURRENT_LIMIT);
    m_bottomFlywheelMotor.setSmartCurrentLimit(FLYWHEEL_CURRENT_LIMIT);
    m_angleMotor.setSmartCurrentLimit(ANGLE_MOTOR_CURRENT_LIMIT);

    // Disable indexer hard limits
    // m_indexerMotor.disableForwardLimitSwitch();
    // m_indexerMotor.disableReverseLimitSwitch();

    // Initialize shooter state
    m_desiredShooterState = getCurrentState();

    // Set maximum shooting distance
    MAX_SHOOTING_DISTANCE = shooterMap.get(shooterMap.size() - 1).getKey();

    // Initialize sim variables
    m_mechanism2d = new Mechanism2d(1.0, 1.0);
  }

  /**
   * Initialize hardware devices for shooter subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware shooterHardware = new Hardware(
      new CANSparkMax(CANIDConstants.kShooterLeftCandId, MotorType.kBrushless),
      new CANSparkMax(CANIDConstants.kShooterRightCandId, MotorType.kBrushless),
      new CANSparkMax(CANIDConstants.kShooterPivotCandId, MotorType.kBrushless),
      new CANSparkMax(CANIDConstants.kShooterFeederCandId, MotorType.kBrushless)
    );

    return shooterHardware;
  }

  /**
   * Set shooter to desired state
   * @param state Desired shooter state
   */
  private void setState(State state) {
    m_desiredShooterState = normalizeState(state);

    m_topFlywheelMotor.set(m_desiredShooterState.speed.in(Units.MetersPerSecond));
    // m_angleMotor.smoothMotion(
    //   m_desiredShooterState.angle.in(Units.Radians),
    //   m_angleConstraint
    // );
  }

  /**
   * Normalize shooter state to be within valid values
   * @param state Desired state
   * @return Valid shooter state
   */
  private State normalizeState(State state) {
    if (state.speed.isNear(ZERO_FLYWHEEL_SPEED, 0.01)) return new State(ZERO_FLYWHEEL_SPEED, state.angle);
    return new State(
      Units.MetersPerSecond.of(MathUtil.clamp(
        state.speed.in(Units.MetersPerSecond),
        -MAX_FLYWHEEL_SPEED.in(Units.MetersPerSecond),
        +MAX_FLYWHEEL_SPEED.in(Units.MetersPerSecond)
      )),
      Units.Radians.of(MathUtil.clamp(
        state.angle.in(Units.Radians),
        m_angleConfig.getOutputMin(), //Changed these to .getOutputMin and .getOutputMax, dont know if thats correct or what it wants
        m_angleConfig.getOutputMax()
      ))
    );
  }

  /**
   * Reset shooter state
   */
  private void resetState() {
    setState(new State(ZERO_FLYWHEEL_SPEED, m_desiredShooterState.angle));
  }

  /**
   * Get current shooter state
   * @return Current shooter state
   */
  private State getCurrentState() {
    return new State(
      Units.MetersPerSecond.of(m_topFlywheelMotor.getEncoder().getVelocity()),
      Units.Radians.of(m_angleMotor.getAbsoluteEncoder().getPosition())
    );
  }

  /**
   * Get shooter state based on distance to target
   * @return Shooter state for current target distance
   */
  private State getAutomaticState() {
    // var targetDistance = getTargetDistance();
    // var flywheelSpeed = m_shooterFlywheelCurve.value(targetDistance.in(Units.Meters));
    var flywheelSpeed = 0.0;
    // var angle = m_shooterAngleCurve.value(targetDistance.in(Units.Meters));
    var angle = 0.0;

    return new State(Units.MetersPerSecond.of(flywheelSpeed), Units.Radians.of(angle));
  }

  /**
   * Get distance to target, clamped to maximum shooting distance
   * @return Distance to target
   */
  private Measure<Distance> getTargetDistance() {
    return Units.Meters.of(
      MathUtil.clamp(
        m_poseSupplier.get().getTranslation().getDistance(m_targetSupplier.get().getSecond()),
        MIN_SHOOTING_DISTANCE.in(Units.Meters),
        MAX_SHOOTING_DISTANCE.in(Units.Meters)
      )
    );
  }

  /**
   * Check if shooter has reached desired state and is ready
   * @return True if ready
   */
  private boolean isReady() {
    // return m_angleMotor.isSmoothMotionFinished() &&
    //   Precision.equals(m_topFlywheelMotor.getInputs().encoderVelocity, m_desiredShooterState.speed.in(Units.MetersPerSecond), m_flywheelConfig.getTolerance());
    return true;
  }

  /**
   * Feed game piece to flywheels
   * @param slow True to run slowly
   */
  private void feedStart(boolean slow) {
    m_indexerMotor.set(slow ? +INDEXER_SLOW_SPEED.in(Units.Percent) : +INDEXER_SPEED.in(Units.Percent));
  }

  /**
   * Reverse indexer
   * @param slow True to run slowly
   */
  private void feedReverse(boolean slow) {
    m_indexerMotor.set(slow ? -INDEXER_SLOW_SPEED.in(Units.Percent) : -INDEXER_SPEED.in(Units.Percent));
  }

  /**
   * Stop feeding
   */
  private void feedStop() {
    m_indexerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // var currentState = getCurrentState();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run in simulation

  }

  /**
   * Intake a game piece from the ground intake to be later fed to the shooter
   * @return Command to intake to the shooter
   */
  public Command intakeCommand() {
    return startEnd(
      () -> {
        feedStart(true);
      },
      () -> {
        feedStop();
      }
    );
  }

  /**
   * Intake a game piece from the source via the shooter
   * @return Command that intakes via the shooter
   */
  public Command sourceIntakeCommand() {
    return startEnd(
      () -> {
        feedReverse(true);
        setState(State.SOURCE_INTAKE_STATE);
      },
      () -> {
        feedStop();
        resetState();
      }
    );
  }

  /**
   * Intake a game piece from the ground intake to be later fed to the shooter with no limit switches
   * @return Command to feed through the shooter
   */
  public Command feedCommand() {
    return startEnd(() -> feedStart(false), () -> feedStop());
  }

  public Command feedThroughCommand(BooleanSupplier isAimed) {
    return startEnd(() -> {
      feedStart(false);
      setState(new State(Units.MetersPerSecond.of(2.0), m_desiredShooterState.angle));
    },
    () -> {
      feedStop();
      resetState();
    });
  }

  /**
   * Reverse note from shooter into intake
   * @return Command to outtake note in shooter
   */
  public Command outtakeCommand() {
    return startEnd(
      () -> feedReverse(false),
      () -> feedStop()
    );
  }

  /**
   * Shoot by manually setting shooter state
   * @param stateSupplier Desired shooter state supplier
   * @return Command to control shooter manually
   */
  public Command shootManualCommand(Supplier<State> stateSupplier) {
    return runEnd(
      () -> {
        if (isReady()) feedStart(false);
        else feedStop();
      },
      () -> {
        feedStop();
        resetState();
      }
    ).beforeStarting(() -> setState(stateSupplier.get()), this);
  }

  /**
   * Shoot by manually setting shooter state
   * @param state Desired shooter state
   * @return Command to control shooter manually
   */
  public Command shootManualCommand(State state) {
    return shootManualCommand(() -> state);
  }

  /**
   * Shoot automatically based on current location
   * @param isAimed Is robot aimed at target
   * @param override Shoot even if target tag is not visible and not in range
   * @return Command to automatically shoot note
   */
  public Command shootCommand(BooleanSupplier isAimed, BooleanSupplier override) {
    return runEnd(
      () -> {
        setState(getAutomaticState());
        if (RobotBase.isSimulation() | isReady()
            && isAimed.getAsBoolean()
            && getTargetDistance().lte(MAX_SHOOTING_DISTANCE) | override.getAsBoolean())
          feedStart(false);
        else feedStop();
      },
      () -> {
        feedStop();
        resetState();
      }
    );
  }

 /**
   * Shoot automatically based on current location, checking if target tag is visible and robot is in range
   * @param isAimed Is robot aimed at target
   * @return Command to automatically shoot note
   */
  public Command shootCommand(BooleanSupplier isAimed) {
    return shootCommand(isAimed, () -> false);
  }

  /**
   * Shoot note into speaker from subwoofer
   * @return Command to shoot note when parked against the subwoofer
   */
  public Command shootSpeakerCommand() {
    return shootManualCommand(State.SPEAKER_SCORE_STATE);
  }

  /**
   * Move shooter to amp position
   * @return Command that prepares shooter for scoring in the amp
   */
  public Command prepareForAmpCommand() {
    return startEnd(
      () -> setState(State.AMP_PREP_STATE),
      () -> resetState()
    ).until(() -> isReady());
  }

  /**
   * Score note in amp
   * @return Command that shoots note into amp
   */
  public Command scoreAmpCommand() {
    return shootManualCommand(State.AMP_SCORE_STATE);
  }

  @Override
  public void close() {
    m_topFlywheelMotor.close();
    m_bottomFlywheelMotor.close();
    m_angleMotor.close();
    m_indexerMotor.close();
  }
}