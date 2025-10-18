package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Preferences;
import frc.robot.statemachines.CoralState;

@Logged
public class Corraler extends SubsystemBase{
    private final TalonFX m_coralMotor;
    private Slot0Configs m_coralSlot0Configs;
    private MotorOutputConfigs m_coralMotorOutputConfigs;

    private final CANrange m_beambreak_enter;
    private final CANrange m_beambreak_prep;

    private final CoralState m_coralState = CoralState.getInstance();

    public Corraler(){
        m_coralMotor = new TalonFX(CorralerConstants.kCoralMotorId);
    
        m_coralSlot0Configs = CorralerConstants.createCoralMotorSlot0Configs();
        m_coralMotor.getConfigurator().apply(m_coralSlot0Configs);
        m_coralMotorOutputConfigs = CorralerConstants.createCoralMotorOutputConfigs();
        m_coralMotor.getConfigurator().apply(m_coralMotorOutputConfigs);

        m_beambreak_enter = new CANrange(CorralerConstants.kEnterBeamBreakId);
        m_beambreak_prep = new CANrange(CorralerConstants.kPrepBeamBreakId);

        configureCANrange();
    }

    public void configureCANrange(){
        ProximityParamsConfigs proximityParamsConfigs = new ProximityParamsConfigs();
        m_beambreak_enter.getConfigurator().refresh(proximityParamsConfigs);
        m_beambreak_enter.getConfigurator().apply(
        proximityParamsConfigs
          .withProximityThreshold(Units.Inches.of(1))
          .withProximityHysteresis(Units.Inches.of(1.25))
          );
      
          
        m_beambreak_prep.getConfigurator().refresh(proximityParamsConfigs);
        m_beambreak_prep.getConfigurator().apply(
        proximityParamsConfigs
          .withProximityThreshold(Units.Inches.of(1))
          .withProximityHysteresis(Units.Inches.of(1.25))
        );
    }

    public void outtakeCoral(){
        m_coralMotor.set(Preferences.coralOuttakePower.getValue());
    }
    
    public void intakeCoral(){
        m_coralMotor.set(Preferences.coralIntakePower.getValue());
    }
    
    public void stopCoralMotor(){
        m_coralMotor.stopMotor();
    }

    @Logged
    public boolean seesCoralEnter(){
        return m_beambreak_enter.getIsDetected().getValue();
    }

    @Logged
    public boolean coralPreped(){
        return m_beambreak_prep.getIsDetected().getValue();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Coral Target", m_coralState.getCoralTargetName());
        m_coralState.setHasCoral(!seesCoralEnter() && coralPreped());
        m_coralState.setBlocked(seesCoralEnter());
    }

    //Voltage, Current, Temperature

    /*********Logging Motors*************/
    @Logged(name = "Motor Voltage", importance = Importance.CRITICAL)
    public double getMotorVoltage(){
        return m_coralMotor.getMotorVoltage().getValueAsDouble();
    }

    @Logged(name = "Motor Current", importance = Importance.CRITICAL)
    public double getMotorCurrent(){
        return m_coralMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Motor Temperature", importance = Importance.CRITICAL)
    public double getMotorTemperature(){
        return m_coralMotor.getDeviceTemp().getValueAsDouble();
    }

}
