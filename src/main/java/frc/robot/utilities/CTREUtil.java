package frc.robot.utilities;

import com.ctre.phoenix.ErrorCode;
import frc.robot.Robot;

import java.util.function.Supplier;

public class CTREUtil {
    /**
     * A recursive function to check if the error code from the errorCodeSupplier is OK.
     * If not, print to the console, and try again, up to attempts times.
     *
     * @param errorCodeSupplier A supplier for a phoenix config function that returns an error code.
     * @param attempts          The number of times to try the function.
     * @return The error code from the last call to the errorCodeSupplier.
     */
    public static ErrorCode checkError(Supplier<ErrorCode> errorCodeSupplier, int attempts) {
        // If this is a simulation, skip this and let the user know
        if(Robot.isSimulation()) {
            System.out.println("Skipping error check because this is a simulation");
            return ErrorCode.OK;
        }
        ErrorCode errorCode = errorCodeSupplier.get();
        if(errorCode != ErrorCode.OK) {
            if(attempts > 0) {
                System.out.println("Error: " + errorCode.toString() + ". Retrying...");
                checkError(errorCodeSupplier, attempts - 1);
            } else {
                System.out.println("Error: " + errorCode.toString() + ". Giving up.");
            }
        }
        return errorCode;
    }

    /**
     * A recursive function to check if the error code from the errorCodeSupplier is OK.
     * If not, print to the console, and try again, up to a default amount of times.
     *
     * @param errorCodeSupplier A supplier for a phoenix config function that returns an error code.
     */
    public static ErrorCode checkError(Supplier<ErrorCode> errorCodeSupplier) {
        return checkError(errorCodeSupplier, 3);
    }
}
