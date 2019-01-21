package frc.robot.util;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Util {

    public static void initTalon(TalonSRX... motors) {
        for (TalonSRX motor : motors) {
            motor.setNeutralMode(NeutralMode.Coast);
            motor.neutralOutput();
            motor.setSensorPhase(false);
            motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
            motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
            motor.configNominalOutputForward(0d, 0);
            motor.configNominalOutputReverse(0d, 0);
            motor.configClosedloopRamp(.5d, 0);
        }
    }

}