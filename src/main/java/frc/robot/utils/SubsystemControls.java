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


    public SubsystemControls(
        @JsonProperty(required = true, value = "launcher") boolean launcher,
        @JsonProperty(required = true, value = "scrap")      boolean scrap,
        @JsonProperty(required = true, value = "salvage")      boolean salvage
        )
    {
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

}
