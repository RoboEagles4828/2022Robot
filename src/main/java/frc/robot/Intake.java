package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants.*;

public class Intake {
    public WPI_VictorSPX intake;

    public Intake(){
        intake = new WPI_VictorSPX(Ports.intake_port);
    }

    public void set_speed(double speed){
        intake.set(speed);
    }

    public void set_voltage(double voltage){
        intake.setVoltage(voltage);
    }

    public void stop(){
        intake.set(0);
    }
}
