package frc.robot.utils;

import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * This class holds the subsystem control values as imported from the subsystem control
 * JSON file. This is made for the 2022 season
 */
public class SubsystemControls
{

    private final boolean launcher;
    private final boolean scrap;
    private final boolean salvage;
    private final boolean mecanum;
    /**
     * Creates a new SubsystemControls object with the specified values.
     * 
     * @param mecanum   true if the mecanum drive subsystem is present and should be enabled;
     *                  false otherwise
     * @param launcher  true if the launcher subsystem is present and should be enabled; false
     *                  otherwise
     * @param scrap     true if the scrap intake subsystem is present and should be enabled;
     *                  false otherwise
     * @param salvage   true if the salvage intake subsystem is present and should be enabled;
     *                  false otherwise
     */
    public SubsystemControls(
        @JsonProperty(required = true, value = "mecanum")      boolean mecanum,
        @JsonProperty(required = true, value = "launcher")     boolean launcher,
        @JsonProperty(required = true, value = "scrap")        boolean scrap,
        @JsonProperty(required = true, value = "salvage")      boolean salvage
        )
    {
        this.mecanum = mecanum;
        this.launcher = launcher;
        this.scrap = scrap;
        this.salvage = salvage;
    }


    /**
     * Returns true if the drive subsystem is indicated as present and should be enabled.
     * 
     * @return true if the drive subsystem is indicated as present and should be enabled; false
     *         otherwise
     */
    public boolean isLauncherPresent() {
        return launcher;
    }
    public boolean isScrapIntakePresent()
    {
        return scrap;
    }
    public boolean isSalvageIntakePresent() {
        return salvage;
    }
    public boolean isMecanumDrivePresent() {
        return mecanum;
    }

}
