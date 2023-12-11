package frc.robot.simulator.hal.rev;

import com.revrobotics.jni.RevJNIWrapper;
import edu.wpi.first.util.RuntimeLoader;

public class SimRevJNIWrapper {
  static boolean libraryLoaded = false;
  static RuntimeLoader<RevJNIWrapper> loader = null;

  static {
    // do nothing in the sim version
  }
}
