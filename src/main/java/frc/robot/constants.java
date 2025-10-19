package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.lib.util.SwerveModuleConstants;
import java.awt.event.KeyListener;

import com.fasterxml.jackson.databind.jsontype.impl.StdTypeResolverBuilder;

public class Constants {
    
    public static class Controller_Costantants{
        public static double stickDeadband = 0;
    }

    public static final class Elevator{
        public static final int L_Elevator =  0;
        public static final int R_Elevator =  0;
    }

    public static final class CanIDs{

    }

    public static final class Climber{

    }

    public static final class Intake{

    }

    public static final class PhotonVision{
        public static final String cameraName = "photonvision_L";
        public static final String cameraName = "photonvision_R";

    }

    public static final class Swerve{

        //Profiling
        private static final double Max_speed = 0; //meters per second
        private static final double Max_Angular_Vel = 0;

        public static final class Module_0{
            
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 0;
            public static final double canCoderOffset = 0; //insert value here

        }

        public static final class Module_1{
            
        }

        public static final class Module_2{
            
        }

        public static final class Module_3{
            
        }

    }


}