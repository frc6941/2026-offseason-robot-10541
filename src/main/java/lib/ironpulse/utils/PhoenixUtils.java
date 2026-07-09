// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package lib.ironpulse.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import java.util.Arrays;
import java.util.function.Supplier;

public class PhoenixUtils {
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }

    /** CAN buses and their signals for synchronized refresh. */
    private static CANBus[] canBuses = new CANBus[0];

    private static BaseStatusSignal[][] signalsByBus = new BaseStatusSignal[0][];

    /** Registers CAN buses so signal groups stay split by physical bus. */
    public static void configureCanBuses(CANBus... buses) {
        for (CANBus bus : buses) {
            getOrCreateBusIndex(bus);
        }
    }

    /** Registers a set of signals for synchronized refresh. */
    public static void registerSignals(CANBus bus, BaseStatusSignal... signals) {
        int busIndex = getOrCreateBusIndex(bus);
        BaseStatusSignal[] existingSignals = signalsByBus[busIndex];
        BaseStatusSignal[] newSignals =
                new BaseStatusSignal[existingSignals.length + signals.length];
        System.arraycopy(existingSignals, 0, newSignals, 0, existingSignals.length);
        System.arraycopy(signals, 0, newSignals, existingSignals.length, signals.length);
        signalsByBus[busIndex] = newSignals;
    }

    /** Refresh all registered signals. */
    public static void refreshAll() {
        for (BaseStatusSignal[] signals : signalsByBus) {
            if (signals.length > 0) {
                BaseStatusSignal.refreshAll(signals);
            }
        }
    }

    private static int getOrCreateBusIndex(CANBus bus) {
        for (int i = 0; i < canBuses.length; i++) {
            if (canBuses[i].equals(bus)) {
                return i;
            }
        }

        canBuses = Arrays.copyOf(canBuses, canBuses.length + 1);
        signalsByBus = Arrays.copyOf(signalsByBus, signalsByBus.length + 1);
        canBuses[canBuses.length - 1] = bus;
        signalsByBus[signalsByBus.length - 1] = new BaseStatusSignal[0];
        return canBuses.length - 1;
    }
}
