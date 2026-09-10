# Sienci ATCi Plugin for grblHAL

This plugin provides advanced safety, state management, and sensor integration for the **[Sienci Automatic Tool Changer (ATC)](https://sienci.com/product/automatic_tool_changer/)**.

The plugin is supported on Longboard32, Longboard32_EXT, and SLB-Lite boards.

Its primary function is to enforce a **Keepout Zone** around the tool rack. This prevents the machine from accidentally jogging or moving into the tool rack during normal operation, while automatically allowing access during tool change macros.

## Features

*   **Keepout Zone:** Defines a safety region (X/Y Min/Max) where the spindle is forbidden to enter during standard operation.
*   **Smart Enforcement:**
    *   **Jog Protection:** Stops jog moves exactly at the boundary of the rack zone.
    *   **G-Code Clipping:** Prevents G-code programs from crossing through the rack area.
*   **Dynamic State Management:** Automatically disables the Keepout Zone when a Tool Change macro is detected, allowing the machine to fetch/deposit tools without triggering alarms.
*   **Sensor Monitoring:** Reports real-time status of ATC sensors to gSender via the `|ATCI:` report string:
    *   Rack Presence (Is the rack mounted?)
    *   Drawbar Status (Open/Closed)
    *   Tool Presence (Is a tool in the spindle?)
    *   Air Pressure
*   **SLB-Lite spindle safety:** Monitors the ATC temperature sensor and raises
    an E-stop if a sensor that was present at startup later disconnects.

## Configuration

The plugin adds the following settings to grblHAL. You must configure the boundaries to match your specific machine and rack location.

| ID | Name | Description | Format |
| :--- | :--- | :--- | :--- |
| **$683** | **ATCi Configuration** | Bitmask flags to enable features. | `Enable, Monitor Rack, Monitor Macro` |
| **$684** | **X Min** | Minimum X coordinate of the Keepout Zone. | `mm` |
| **$685** | **Y Min** | Minimum Y coordinate of the Keepout Zone. | `mm` |
| **$686** | **X Max** | Maximum X coordinate of the Keepout Zone. | `mm` |
| **$687** | **Y Max** | Maximum Y coordinate of the Keepout Zone. | `mm` |

### Setting $683 Flags
*   **Enable:** Master switch for the plugin.
*   **Monitor Rack Presence:** If enabled, the Keepout Zone is only active if the Rack Sensor detects the rack is installed. On Longboard boards this is Aux Input 7; on SLB-Lite it is MCP23017 input port 0.
*   **Monitor TC Macro:** If enabled, the plugin automatically disables the Keepout Zone when it detects a Tool Change macro running.

## Usage

### M960 Command
The plugin introduces `M960` to manually control the Keepout Zone enforcement state at runtime.

*   `M960 P1`: **Enable** Keepout Zone enforcement.
*   `M960 P0`: **Disable** Keepout Zone enforcement.

*(Note: This overrides the current state until the next manual/automatic state change.)*

## Status Reporting

The plugin appends a status string to the grblHAL realtime report (e.g., `|ATCI:SEZB`).

**Legend:**
*   `E` - Keepout Enforcement is **Enabled**.
*   `Z` - Machine is currently **Inside** the Keepout Zone.
*   `R` / `M` / `T` / `S` - Source of the current state (Rack Sensor, M-Code, Tool Macro, Startup).
*   `I` - Rack is Installed
*   `B` - Drawbar is Open
*   `L` - Tool is Loaded
*   `P` - Air Pressure

## Pin Mappings (Default)

| Function | Port/Pin | Description |
| :--- | :--- | :--- |
| **Rack Presence** | `AUXINPUT7` | Detects if the tool rack is physically mounted. |
| **Drawbar** | `AUXINPUT0` | Detects drawbar position. |
| **Tool Sensor** | `AUXINPUT1` | Detects if a tool is in the collet. |
| **Pressure** | `AUXINPUT2` | Monitors air pressure. |

### SLB-Lite MCP23017 Mapping

On SLB-Lite, the ATC sensors use MCP23017 input ports without claiming the
ports:

| MCP Input | Description |
| :--- | :--- |
| 0 | Sienci ATC Rack |
| 1 | Sienci ATC Drawbar Sensor |
| 2 | Sienci ATC Tool Sensor |
| 3 | Sienci ATC Temperature Sensor |
| 4 | Sienci ATC Pressure Sensor |

MCP23017 output ports 0 through 2 are named `Sienci ATC Drawbar Enable`,
`Sienci ATC Drawbar`, and `Sienci ATC Airseal`. The plugin does not claim these
ports.

The temperature sensor is sampled at startup. If it is low at startup, the
plugin assumes no spindle is present and does not arm the safety check. If it
starts high and later goes low, the plugin raises an E-stop and reports
`Spindle Overheated or Temp Sensor disconnected`.
