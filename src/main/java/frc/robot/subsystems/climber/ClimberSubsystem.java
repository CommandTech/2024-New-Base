// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class ClimberSubsystem extends SubsystemBase {
  public static class Hardware {
    private CANSparkMax winchMotor;
    private CANSparkMax elevatorMotor;

    public Hardware(CANSparkMax winchMotor, CANSparkMax elevatorMotor) {
      this.winchMotor = winchMotor;
      this.elevatorMotor = elevatorMotor;
    }
  }

  private CANSparkMax m_winchMotor;
  private CANSparkMax m_elevatorMotor;

  private final Measure<Dimensionless> ELEVATOR_VELOCITY;
  private final Measure<Dimensionless> WINCH_VELOCITY;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(Hardware climberHardware, Measure<Dimensionless> elevatorVelocity, Measure<Dimensionless> winchVelocity) {
    this.m_winchMotor = climberHardware.winchMotor;
    this.m_elevatorMotor = climberHardware.elevatorMotor;

    //Do not know what these are, but spark maxes do not have a method for these
    // m_winchMotor.enableReverseLimitSwitch();
    // m_elevatorMotor.enableReverseLimitSwitch();

    ELEVATOR_VELOCITY = elevatorVelocity;
    WINCH_VELOCITY = winchVelocity;
  }

  /**
   * Initialize hardware devices for climber subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware climberHardware = new Hardware(
      new CANSparkMax(CANIDConstants.kWinchCandId, MotorType.kBrushless),
      new CANSparkMax(CANIDConstants.kClimberCandId, MotorType.kBrushless)
    );

    return climberHardware;
  }

  // Raises the elevator
  private void raiseElevator() {
    m_elevatorMotor.set(ELEVATOR_VELOCITY.in(Units.Percent));
  }

  // Lowers the elevator
  private void lowerElevator() {
    m_elevatorMotor.set(-ELEVATOR_VELOCITY.in(Units.Percent));
  }

  // Reel in winch
  private void reelWinch() {
    m_winchMotor.set(-WINCH_VELOCITY.in(Units.Percent));
  }

  // Unreels the winch
  private void unreelWinch() {
    m_winchMotor.set(WINCH_VELOCITY.in(Units.Percent));
  }

  // Stop elevator motor
  private void stopElevator() {
    m_elevatorMotor.stopMotor();
  }

  // Stop winch motor
  private void stopWinch() {
    m_winchMotor.stopMotor();
  }

  /**
   * Raise the elevator
   * @return Command to run the elevator motor
   */
  public Command raiseElevatorCommand() {
    return startEnd(() -> raiseElevator(), () -> stopElevator());
  }

  /**
   * Lower the elevator
   * @return Command to run the elevator motor in the reverse direction
   */
  public Command lowerElevatorCommand() {
    return startEnd(() -> lowerElevator(), () -> stopElevator());
  }

  /**
   * Reel in the winch
   * @return Command to run the winch motor
   */
  public Command reelWinchCommand() {
    return startEnd(() -> reelWinch(), () -> stopWinch());
  }

  /**
   * Unreel the winch
   * @return Command to run the winch motor in the reverse direction
   */
  public Command unreelWinchCommand() {
    return startEnd(() -> unreelWinch(), () -> stopWinch());
  }

  @Override
  public void periodic() {
    m_elevatorMotor.stopMotor();
  }
}