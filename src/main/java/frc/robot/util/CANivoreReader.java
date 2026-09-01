// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class CANivoreReader {
  private final CANBus canBus;
  private final String logPrefix;
  private final Thread thread;
  private Optional<CANBusStatus> status = Optional.empty();

  public CANivoreReader(String canBusName) {
    canBus = new CANBus(canBusName);
    logPrefix = "CANivoreStatus/" + canBusName + "/";
    thread =
        new Thread(
            () -> {
              while (true) {
                var statusTemp = Optional.of(canBus.getStatus());
                synchronized (this) {
                  status = statusTemp;
                }
                try {
                  Thread.sleep(400); // Match RIO CAN sampling
                } catch (InterruptedException e) {
                  e.printStackTrace();
                }
              }
            });
    thread.setName("CANivoreReader");
    thread.start();
  }

  public synchronized Optional<CANBusStatus> getStatus() {
    return status;
  }

  public void logStatus() {
    var canivoreStatus = getStatus();
    if (canivoreStatus.isPresent()) {
      Logger.recordOutput(logPrefix + "Status", canivoreStatus.get().Status.getName());
      Logger.recordOutput(logPrefix + "Utilization", canivoreStatus.get().BusUtilization);
      Logger.recordOutput(logPrefix + "OffCount", canivoreStatus.get().BusOffCount);
      Logger.recordOutput(logPrefix + "TxFullCount", canivoreStatus.get().TxFullCount);
      Logger.recordOutput(logPrefix + "ReceiveErrorCount", canivoreStatus.get().REC);
      Logger.recordOutput(logPrefix + "TransmitErrorCount", canivoreStatus.get().TEC);
    }
  }
}
