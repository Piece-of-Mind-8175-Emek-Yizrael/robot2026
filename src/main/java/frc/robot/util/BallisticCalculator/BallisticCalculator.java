package frc.robot.util.BallisticCalculator;

import java.io.File;
import java.util.Scanner;

public class BallisticCalculator {
    public BallisticCalculator(String shooterTablePath) {
        try {
            Scanner scanner = new Scanner(new File(shooterTablePath));
            StringBuilder fileContents = new StringBuilder();
            while(scanner.hasNextLine()) {
                fileContents.append(scanner.nextLine());
            }
        } catch (Exception e) {
            // log the error message
        }
    }

    BallisticCalculatorEntry getBest(BallisticCalculatorEntry currentState) {
        // add a loss function to decide which angle and velocity are best (considering the current state)

        return null;
    }
}
