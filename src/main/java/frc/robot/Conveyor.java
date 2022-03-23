package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.*;

public class Conveyor {
    public WPI_TalonSRX conveyor;

    public Conveyor(){
        conveyor = new WPI_TalonSRX(Ports.conveyor_port);
    }

    public void set_speed(double speed){
        conveyor.set(speed);
    }

    public void set_voltage(double voltage){
        conveyor.setVoltage(voltage);
    }

    public void stop(){
        conveyor.set(0);
    }
}
