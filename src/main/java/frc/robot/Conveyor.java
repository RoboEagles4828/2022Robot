package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Conveyor {
    public WPI_TalonSRX conveyor;

    public Conveyor(){
        conveyor = new WPI_TalonSRX(Constants.conveyor_port);
    }

    public void set_speed(double speed){
        conveyor.set(speed);
    }

    public void stop(){
        conveyor.set(0);
    }
}
