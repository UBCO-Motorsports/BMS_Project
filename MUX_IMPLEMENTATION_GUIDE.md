# PATH B Implementation Guide - 19 Sensors with MUX Control

## Overview
This guide provides step-by-step instructions to enable MUX-based temperature sensing for 19 sensors per BQ79616 device. You will modify 4 files to implement this functionality.

**Goal:** Read 19 temperature sensors per BQ79616 device:
- 16 sensors via 2× 8:1 multiplexers (GPIO1 and GPIO2)
- 3 direct sensors (GPIO3, GPIO4, GPIO5)
- MUX control via GPIO6, GPIO7, GPIO8

---

## Hardware Architecture

```
BQ79616 Device (per device in stack):
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  GPIO1 ──────► 8:1 MUX ───► [8 NTC thermistors] (sensors 0-7) │
│                  │                                              │
│  GPIO2 ──────► 8:1 MUX ───► [8 NTC thermistors] (sensors 8-15)│
│                  │                                              │
│  GPIO3 ──────────────────► [NTC thermistor] (sensor 16)       │
│  GPIO4 ──────────────────► [NTC thermistor] (sensor 17)       │
│  GPIO5 ──────────────────► [NTC thermistor] (sensor 18)       │
│                  │                                              │
│  GPIO6 ─────► S0 ├─────► MUX Control (channel select bit 0)   │
│  GPIO7 ─────► S1 ├─────► MUX Control (channel select bit 1)   │
│  GPIO8 ─────► S2 └─────► MUX Control (channel select bit 2)   │
│                                                                 │
│  Total: 8 + 8 + 3 = 19 temperature sensors                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### MUX Channel Select Truth Table
| Ch# | S2 | S1 | S0 | Selected Input |
|-----|----|----|----| ---------------|
| 0   | 0  | 0  | 0  | MUX Input 0    |
| 1   | 0  | 0  | 1  | MUX Input 1    |
| 2   | 0  | 1  | 0  | MUX Input 2    |
| 3   | 0  | 1  | 1  | MUX Input 3    |
| 4   | 1  | 0  | 0  | MUX Input 4    |
| 5   | 1  | 0  | 1  | MUX Input 5    |
| 6   | 1  | 1  | 0  | MUX Input 6    |
| 7   | 1  | 1  | 1  | MUX Input 7    |

---

## File Changes Required

### Summary
- **File 1:** `include/config.h` - Update TEMP_SENSORS_PER_SLAVE
- **File 2:** `src/bq_comm.cpp` - Fix GPIO configuration and update MUX function
- **File 3:** `include/bq_comm.h` - Update function signature
- **File 4:** `src/bq_data.cpp` - Complete rewrite of temperature reading logic

---

## FILE 1: include/config.h

### Change 1: Update TEMP_SENSORS_PER_SLAVE

**Location:** Line 10

**Current:**
```cpp
#define TEMP_SENSORS_PER_SLAVE 8
```

**Change to:**
```cpp
#define TEMP_SENSORS_PER_SLAVE 19
```

**Explanation:** This increases the temperature array size from 8 to 19 sensors per device.

---

## FILE 2: src/bq_comm.cpp

### Change 1: Fix GPIO Configuration (Lines 566-573)

**Location:** Inside the `configure_stack()` function

#### Modify GPIO_CONF3 (Line 570)

**Current:**
```cpp
status = bqStackWrite(GPIO_CONF3, 0x09, 1);
```

**Change to:**
```cpp
status = bqStackWrite(GPIO_CONF3, 0x29, 1);
```

**Explanation:**
- `0x09` = binary `0b001001` = GPIO6 as temp input, GPIO5 as temp input
- `0x29` = binary `0b101001` = GPIO6 as output LOW, GPIO5 as temp input
- This configures GPIO6 as a digital output for MUX control (S0 pin)

**Register Structure:**
```
GPIO_CONF3 bits: [7:6 unused] [5:3 = GPIO6] [2:0 = GPIO5]
  0b001 = Temperature measurement mode
  0b101 = Digital output LOW
  0x29 = GPIO6 output (0b101) | GPIO5 temp (0b001)
```

#### Modify GPIO_CONF4 (Line 572)

**Current:**
```cpp
status = bqStackWrite(GPIO_CONF4, 0x09, 1);
```

**Change to:**
```cpp
status = bqStackWrite(GPIO_CONF4, 0x2D, 1);
```

**Explanation:**
- `0x09` = binary `0b001001` = GPIO8 as temp input, GPIO7 as temp input
- `0x2D` = binary `0b101101` = GPIO8 as output LOW, GPIO7 as output LOW
- This configures GPIO7 and GPIO8 as digital outputs for MUX control (S1 and S2 pins)

**Register Structure:**
```
GPIO_CONF4 bits: [7:6 unused] [5:3 = GPIO8] [2:0 = GPIO7]
  0b101 = Digital output LOW
  0x2D = GPIO8 output (0b101) | GPIO7 output (0b101)
```

### Change 2: Update bqSetMuxChannel Function Signature (Line 513)

**Current function signature:**
```cpp
BMSErrorCode_t bqSetMuxChannel(uint8_t channel) {
```

**Change to:**
```cpp
BMSErrorCode_t bqSetMuxChannel(uint8_t deviceID, uint8_t channel) {
```

**Then inside the function body (Line 523):**

**Current:**
```cpp
bool all_ok = true;
BMSErrorCode_t status;
// Assuming the MUX control pins are physically connected to the first BQ79616 (Device ID 1)
uint8_t deviceID = 1;
```

**Change to:**
```cpp
if (channel > 7) {
    BMS_DEBUG_PRINTF("Invalid MUX channel %d (must be 0-7)\n", channel);
    return BMS_ERROR_UNKNOWN;
}

bool all_ok = true;
BMSErrorCode_t status;
```

**Explanation:** Remove the hardcoded `deviceID = 1` line since deviceID is now a parameter, allowing you to control the MUX on any BQ79616 device in the stack. Add channel validation at the start.

---

## FILE 3: include/bq_comm.h

### Change 1: Update Function Declaration

**Location:** Line 28

**Current:**
```cpp
BMSErrorCode_t bqSetMuxChannel(uint8_t channel);
```

**Change to:**
```cpp
BMSErrorCode_t bqSetMuxChannel(uint8_t deviceID, uint8_t channel);
```

**Explanation:** This matches the new function signature in bq_comm.cpp.

---

## FILE 4: src/bq_data.cpp - MAJOR REWRITE

### Complete Rewrite: bqGetAllTemperatures() Function

**Location:** Starting at line 63

**Current Implementation:** Lines 65-99 read all GPIO temperatures in one bulk read. This doesn't work with MUX control.

**Delete:** Lines 65-99 (keep the function signature and the die temperature reading section starting at line 100)

### New Implementation Structure

#### Part 1: Define Constants (Top of Function)

Add these constant definitions after the function signature and before any logic:

```cpp
const uint8_t NUM_DIRECT_SENSORS = 3;    // GPIO3, GPIO4, GPIO5
const uint8_t NUM_MUX_CHANNELS = 8;      // 8:1 MUX
const uint8_t NUM_MUXES = 2;             // On GPIO1 and GPIO2
```

Keep the existing Steinhart-Hart coefficients:
```cpp
const float a1 = 0.003354016f;
const float b1 = 0.000256524f;
const float c1 = 2.61E-6f;
const float d1 = 6.33E-8f;
const float REF_RESISTOR = 10000.0f;
const float REF_VOLTAGE = 5.0f;
```

Declare variables you'll need:
```cpp
BMSErrorCode_t status;
uint8_t rawGpioData[16];  // Buffer for reading GPIO values
```

#### Part 2: Outer Loop - Iterate Through Devices

Create a loop that processes each BQ79616 device in your stack:

```
For each deviceIdx from 0 to (NUM_BQ79616_DEVICES - 1):
    Calculate deviceID = deviceIdx + 1
        Note: Device addresses start at 1, not 0
    
    Initialize tempIndex = 0
        Note: This tracks which temperature array slot we're filling
```

**Key Point:** `deviceIdx` is 0-based for array indexing, but `deviceID` is 1-based for BQ79616 addressing.

#### Part 3: Read MUX-Based Temperatures (Nested Loops)

This is the core of the new implementation. You'll loop through each MUX and each channel:

```
For each muxIdx from 0 to (NUM_MUXES - 1):
    
    Calculate which GPIO this MUX is connected to:
        gpioNum = muxIdx + 1
        (This gives GPIO1 for muxIdx=0, GPIO2 for muxIdx=1)
    
    For each channel from 0 to (NUM_MUX_CHANNELS - 1):
        
        // STEP A: Set MUX Channel
        Call bqSetMuxChannel(deviceID, channel)
        Check if status != BMS_OK:
            If failed, print debug message and return status
        
        // STEP B: Wait for MUX Settling
        Call delayMicroseconds(500)
            Note: This allows the MUX to stabilize after switching
        
        // STEP C: Read the GPIO Connected to This MUX
        Calculate register address:
            gpioRegAddr = GPIO1_HI + (gpioNum - 1) * 2
            (GPIO1_HI = 0x58E, GPIO2_HI = 0x590, etc.)
        
        Call bqReadReg(deviceID, gpioRegAddr, rawGpioData, 2, FRMWRT_SGL_R)
        Check if status != BMS_OK:
            If failed, print debug message and return status
        
        // STEP D: Convert Raw ADC to Temperature
        Extract bytes:
            msb = rawGpioData[0]
            lsb = rawGpioData[1]
        
        Combine into 16-bit value:
            raw_data = (msb << 8) | lsb
            Note: Cast as uint16_t
        
        Convert to voltage:
            temp_voltage = (float)raw_data * 0.15259
            Note: This is in millivolts, LSB = 152.59 µV
        
        Check for open circuit:
            If temp_voltage >= 4800.0:
                Store 255 in cellTemperatures[tempIndex]
                Note: 255 indicates open circuit or disconnected sensor
            Else:
                Convert voltage to actual voltage in volts:
                    voltage = temp_voltage / 1000.0
                
                Calculate NTC resistance:
                    resistance = (REF_RESISTOR * voltage) / (REF_VOLTAGE - voltage)
                    Note: Standard voltage divider formula
                
                Calculate natural log:
                    log_r = logf(resistance / REF_RESISTOR)
                    Note: Use logf() for float precision
                
                Apply Steinhart-Hart equation:
                    temp_c = 1.0 / (a1 + b1*log_r + c1*log_r^2 + d1*log_r^3) - 273.15
                    Note: Use powf(log_r, 2) and powf(log_r, 3)
                
                Convert to 0.1°C units and store:
                    cellTemperatures[tempIndex] = (int16_t)(temp_c * 10.0)
                    Note: Multiply by 10 to store in tenths of degrees
        
        Increment tempIndex
```

**After this section completes:** `tempIndex` should equal 16 (8 channels × 2 MUXes)

#### Part 4: Read Direct Temperature Sensors

Read GPIO3, GPIO4, and GPIO5 directly without MUX control:

```
Read GPIO3-5 in one command:
    Call bqReadReg(deviceID, GPIO3_HI, rawGpioData, 6, FRMWRT_SGL_R)
        Note: 6 bytes = 3 GPIOs × 2 bytes each
    Check if status != BMS_OK:
        If failed, print debug message and return status

For each i from 0 to (NUM_DIRECT_SENSORS - 1):
    
    Extract bytes from buffer:
        msb = rawGpioData[i * 2]
        lsb = rawGpioData[i * 2 + 1]
    
    Combine into 16-bit value:
        raw_data = (msb << 8) | lsb
    
    Convert to voltage:
        temp_voltage = (float)raw_data * 0.15259
    
    Check for open circuit and convert to temperature:
        Use the SAME logic as in Part 3 Step D
        (Check if >= 4800.0, if not apply Steinhart-Hart)
    
    Store in cellTemperatures[tempIndex]
    
    Increment tempIndex
```

**After this section completes:** `tempIndex` should equal 19

#### Part 5: Read Die Temperatures

**KEEP EXISTING CODE** - Lines 100-113 are correct, do NOT modify:

```cpp
// Read die temperatures after GPIO temperatures
// Reading from DIETEMP1_LO means data comes as [LO, HI] byte order
status = bqStackRead(DIETEMP1_LO, rawTempData, 2 * NUM_BQ79616_DEVICES, 
                     SERIAL_TIMEOUT_MS * NUM_BQ79616_DEVICES);
if (status != BMS_OK) {
    BMS_DEBUG_PRINTLN("Failed to read die temperatures from stack.");
    return status;
}
for (int i = 0; i < NUM_BQ79616_DEVICES; ++i) {
    int raw_idx = i * 2;
    uint8_t lsb = rawTempData[raw_idx];
    uint8_t msb = rawTempData[raw_idx + 1];
    int16_t raw_dietemp = (int16_t)((msb << 8) | lsb);
    bmsData->modules[i].dieTemperature = (int16_t)((float)raw_dietemp * 0.025f * 10.0f);
}
```

#### Part 6: Process Results

**KEEP EXISTING CODE** - Final line of function:

```cpp
processRawTemperatures(bmsData);
return BMS_OK;
```

---

## Implementation Details

### Register Address Calculations

The BQ79616 GPIO registers are sequential:

| GPIO | HI Address | LO Address | Offset from GPIO1_HI |
|------|-----------|------------|---------------------|
| GPIO1 | 0x58E | 0x58F | +0 bytes |
| GPIO2 | 0x590 | 0x591 | +2 bytes |
| GPIO3 | 0x592 | 0x593 | +4 bytes |
| GPIO4 | 0x594 | 0x595 | +6 bytes |
| GPIO5 | 0x596 | 0x597 | +8 bytes |
| GPIO6 | 0x598 | 0x599 | +10 bytes |
| GPIO7 | 0x59A | 0x59B | +12 bytes |
| GPIO8 | 0x59C | 0x59D | +14 bytes |

**Formula:** `GPIOx_HI = GPIO1_HI + (x - 1) * 2`

Example: For GPIO2: `0x58E + (2-1)*2 = 0x58E + 2 = 0x590`

### Timing Considerations

**MUX Settling Time:**
- Current implementation: 500 microseconds
- This is typically sufficient for most analog MUX ICs
- If you see noisy readings, increase to 1000 microseconds
- Check your specific MUX IC datasheet for recommended settling time

**Total Scan Time per Device:**
- Per MUX channel: 500µs settling + ~1ms read = 1.5ms
- Total for 16 MUX channels: 16 × 1.5ms = 24ms
- Plus direct sensors: ~2ms
- Plus die temp: ~1ms
- **Total per device: ~27ms**
- **For 2 devices: ~54ms total**

This is acceptable for typical BMS update rates (100-200ms).

### Temperature Conversion Formula

**ADC to Voltage:**
```
temp_voltage_mV = raw_adc_value × 0.15259 µV/LSB × 1000
temp_voltage_mV = raw_adc_value × 0.15259
```

**Voltage Divider (NTC + Reference Resistor):**
```
V_GPIO = V_REF × (R_NTC / (R_NTC + R_REF))

Rearranged to solve for R_NTC:
R_NTC = (R_REF × V_GPIO) / (V_REF - V_GPIO)
```

**Steinhart-Hart Equation:**
```
1/T = A + B×ln(R) + C×ln(R)² + D×ln(R)³
T_kelvin = 1 / (A + B×ln(R/R_ref) + C×ln(R/R_ref)² + D×ln(R/R_ref)³)
T_celsius = T_kelvin - 273.15
```

Where:
- A = 0.003354016 (a1)
- B = 0.000256524 (b1)
- C = 2.61E-6 (c1)
- D = 6.33E-8 (d1)
- R_ref = 10000Ω (NTC_R_REF)

### Error Handling Best Practices

1. **Check Every Function Return:**
   - After `bqSetMuxChannel()` - ensures MUX switched properly
   - After `bqReadReg()` - ensures valid data was received
   - After `bqStackRead()` - ensures communication succeeded

2. **Return Immediately on Error:**
   - Don't continue processing if a read fails
   - Return the error status to caller
   - Let higher-level code decide how to handle

3. **Add Debug Messages:**
   - Use `BMS_DEBUG_PRINTF()` to log which operation failed
   - Include device ID, channel number, and error status
   - This makes hardware debugging much easier

4. **Validate Data Ranges:**
   - Check if `temp_voltage >= 4800.0` (open circuit)
   - Optionally check if temperature is reasonable (-40°C to +125°C)
   - Store error codes (255) for invalid readings

### Temperature Array Indexing

The `cellTemperatures` array for each device is filled in this order:

| Array Index | Source | Description |
|-------------|--------|-------------|
| 0-7 | GPIO1 MUX | Channels 0-7 from first MUX |
| 8-15 | GPIO2 MUX | Channels 0-7 from second MUX |
| 16 | GPIO3 Direct | Direct sensor on GPIO3 |
| 17 | GPIO4 Direct | Direct sensor on GPIO4 |
| 18 | GPIO5 Direct | Direct sensor on GPIO5 |

**Total: 19 sensors** (matches `TEMP_SENSORS_PER_SLAVE`)

---

## Testing Procedure

### Step 1: Compile Test
```bash
# Build the project
pio run

# Check for compilation errors
# Should compile with no errors or warnings
```

**Expected Result:** Clean compilation, no errors

### Step 2: GPIO Configuration Verification

After calling `configure_stack()`, read back the GPIO_CONF registers:

```cpp
uint8_t conf3_val, conf4_val;
bqReadReg(deviceID, GPIO_CONF3, &conf3_val, 1, FRMWRT_SGL_R);
bqReadReg(deviceID, GPIO_CONF4, &conf4_val, 1, FRMWRT_SGL_R);

Serial.printf("GPIO_CONF3 = 0x%02X (should be 0x29)\n", conf3_val);
Serial.printf("GPIO_CONF4 = 0x%02X (should be 0x2D)\n", conf4_val);
```

**Expected Result:**
- GPIO_CONF3 = 0x29
- GPIO_CONF4 = 0x2D

### Step 3: MUX Control Signal Test

Use an oscilloscope or multimeter to verify MUX control signals change:

```cpp
// Test each channel
for (uint8_t ch = 0; ch < 8; ch++) {
    bqSetMuxChannel(deviceID, ch);
    delay(100);  // Long enough to see on scope
    Serial.printf("Channel %d set\n", ch);
}
```

**Expected Result:**
- GPIO6 (S0) toggles: 0,1,0,1,0,1,0,1
- GPIO7 (S1) toggles: 0,0,1,1,0,0,1,1
- GPIO8 (S2) toggles: 0,0,0,0,1,1,1,1

### Step 4: Temperature Reading Test

Read all temperatures and verify values are realistic:

```cpp
status = bqGetAllTemperatures(&g_bmsData);
if (status == BMS_OK) {
    for (int dev = 0; dev < NUM_BQ79616_DEVICES; dev++) {
        Serial.printf("Device %d temperatures:\n", dev);
        for (int i = 0; i < TEMP_SENSORS_PER_SLAVE; i++) {
            float temp = g_bmsData.modules[dev].cellTemperatures[i] / 10.0f;
            Serial.printf("  Sensor %2d: %.1f°C\n", i, temp);
        }
    }
}
```

**Expected Result:**
- No sensors read 0°C (unless actually at 0°C)
- No sensors read 255 (unless disconnected)
- Values in realistic range (e.g., 15-35°C at room temperature)
- All 19 sensors report values

### Step 5: Known Temperature Test

Apply known temperatures to specific sensors:

1. **Ice Water Test (0°C):**
   - Immerse NTC in ice water
   - Should read 0-2°C

2. **Room Temperature Test (~25°C):**
   - Leave NTC at ambient
   - Should read 20-30°C depending on room

3. **Hot Water Test (~60°C):**
   - Immerse NTC in hot water
   - Should read 55-65°C

**Expected Result:** Readings within ±2°C of known temperature

### Step 6: Settling Time Adjustment Test

If you see unstable readings:

```cpp
// Increase settling time in bqGetAllTemperatures():
delayMicroseconds(500);  // Try 1000 instead
```

**Symptoms of insufficient settling:**
- Readings jump between channels
- Adjacent sensors show identical or very similar values
- Values change when you modify loop order

**Solution:** Increase delay until readings stabilize

### Step 7: Channel Crosstalk Test

Verify channels are isolated:

1. Connect sensors to alternating channels (0, 2, 4, 6)
2. Leave other channels disconnected
3. Read all channels
4. Disconnected channels should read 255 (open circuit)
5. Connected channels should read valid temperatures

**Expected Result:** No crosstalk between channels

---

## Common Mistakes and Solutions

### Mistake 1: Forgetting to Switch Channels

**Symptom:** All 8 sensors on a MUX read the same value

**Cause:** Not calling `bqSetMuxChannel()` or calling it after the read instead of before

**Solution:** Ensure `bqSetMuxChannel()` is called BEFORE `bqReadReg()` for each channel

### Mistake 2: Insufficient Settling Time

**Symptom:** Adjacent sensors show similar values, readings are unstable

**Cause:** MUX hasn't settled before reading

**Solution:** Increase `delayMicroseconds()` from 500 to 1000 or more

### Mistake 3: Wrong Register Addresses

**Symptom:** All temperatures read 0 or garbage values

**Cause:** Calculating GPIO register address incorrectly

**Solution:** Verify formula: `GPIO1_HI + (gpioNum - 1) * 2`
- GPIO1: 0x58E + 0 = 0x58E ✓
- GPIO2: 0x58E + 2 = 0x590 ✓

### Mistake 4: Buffer Overflow

**Symptom:** Crashes or corrupted data

**Cause:** Array index out of bounds

**Solution:** Verify `tempIndex` never exceeds 18 (max index for 19 sensors)

### Mistake 5: Using bqStackRead for MUX Sensors

**Symptom:** Only reads channel 0 of MUX repeatedly

**Cause:** `bqStackRead()` doesn't allow channel switching between devices

**Solution:** Use `bqReadReg()` with `FRMWRT_SGL_R` for individual devices

### Mistake 6: Device ID Off by One

**Symptom:** Communication errors, timeout failures

**Cause:** Using 0-based deviceIdx instead of 1-based deviceID

**Solution:** Always use `deviceID = deviceIdx + 1` for BQ79616 addressing

### Mistake 7: Reusing Buffer Without Clearing

**Symptom:** Occasional wrong readings, especially after errors

**Cause:** Old data left in `rawGpioData` buffer

**Solution:** Either create new buffer for each read, or memset to 0 before each use

---

## Hardware Requirements Checklist

Before implementation, verify your hardware has:

- [ ] 2× 8:1 analog multiplexers per BQ79616
  - Recommended: CD4051, ADG708, MAX4617, or similar
  - Must be analog MUX, not digital
  - Must handle 0-5V signal range

- [ ] MUX1 output connected to BQ79616 GPIO1
- [ ] MUX2 output connected to BQ79616 GPIO2

- [ ] MUX control connections:
  - [ ] Both MUX S0 pins → BQ79616 GPIO6
  - [ ] Both MUX S1 pins → BQ79616 GPIO7
  - [ ] Both MUX S2 pins → BQ79616 GPIO8

- [ ] 16 NTC thermistors on MUX inputs:
  - [ ] 8× NTC on MUX1 inputs (IN0-IN7)
  - [ ] 8× NTC on MUX2 inputs (IN0-IN7)

- [ ] 3 direct NTC thermistors:
  - [ ] 1× NTC on GPIO3
  - [ ] 1× NTC on GPIO4
  - [ ] 1× NTC on GPIO5

- [ ] All NTCs same specification:
  - [ ] 10kΩ @ 25°C (recommended)
  - [ ] Beta value matches Steinhart-Hart coefficients in code
  - [ ] Connected in voltage divider with 10kΩ reference resistor

- [ ] Power supply:
  - [ ] TSREF enabled (provides 5V reference, configured in code)
  - [ ] Reference resistors connected between TSREF and GPIO pins

---

## Debugging Tips

### Enable Verbose Logging

Add detailed logging to help debug issues:

```cpp
#define TEMP_READING_DEBUG  // Add this to config.h

// In bqGetAllTemperatures():
#ifdef TEMP_READING_DEBUG
    BMS_DEBUG_PRINTF("Device %d, MUX %d, Channel %d: raw=0x%04X, voltage=%.1fmV, temp=%.1fC\n",
                     deviceID, muxIdx, channel, raw_data, temp_voltage, temp_c);
#endif
```

### Verify Communication

Test basic communication before implementing full logic:

```cpp
// Test: Can we read GPIO1?
uint8_t test_buf[2];
status = bqReadReg(1, GPIO1_HI, test_buf, 2, FRMWRT_SGL_R);
Serial.printf("GPIO1 read status: %d, value: 0x%02X%02X\n", 
              status, test_buf[0], test_buf[1]);
```

### Verify MUX Switching

Add logging to `bqSetMuxChannel()`:

```cpp
BMS_DEBUG_PRINTF("MUX: Dev=%d, Ch=%d, S2=%d, S1=%d, S0=%d\n",
                 deviceID, channel,
                 (channel & 0b100) ? 1 : 0,
                 (channel & 0b010) ? 1 : 0,
                 (channel & 0b001) ? 1 : 0);
```

### Use Oscilloscope

Monitor signals to verify operation:
- **GPIO6/7/8:** Should show digital transitions when switching channels
- **GPIO1/2:** Should show analog voltage changes as different sensors are selected
- **TSREF:** Should be steady 5.0V

### Check Intermediate Values

Print each step of temperature conversion:

```cpp
Serial.printf("raw_data=%u, temp_voltage=%.2f, voltage=%.4f, resistance=%.1f, temp=%.2f\n",
              raw_data, temp_voltage, voltage, resistance, temp_c);
```

### Verify Array Bounds

Add assertions to catch indexing errors:

```cpp
if (tempIndex >= TEMP_SENSORS_PER_SLAVE) {
    Serial.printf("ERROR: tempIndex overflow! tempIndex=%d\n", tempIndex);
    return BMS_ERROR_UNKNOWN;
}
```

---

## Performance Optimization (Optional)

Once working, consider these optimizations:

### 1. Reduce Settling Time (If Stable)

Test with progressively lower settling times:
- Start at 500µs (conservative)
- Try 300µs, then 200µs, then 100µs
- Use lowest value that gives stable readings
- This reduces total scan time

### 2. Parallel MUX Control

If your hardware supports it, control both MUXes simultaneously:
- Both MUXes share S0/S1/S2 signals
- Set channel once, read both GPIO1 and GPIO2
- Halves the number of channel switches

### 3. Skip Known Disconnected Sensors

If certain channels have no sensor:
- Add a bitmask of connected channels
- Skip `bqSetMuxChannel()` and read for disconnected channels
- Saves time on unused channels

### 4. Use Stack Read for Direct Sensors

Read GPIO3-5 from all devices at once:
```cpp
uint8_t direct_buf[6 * NUM_BQ79616_DEVICES];
status = bqStackRead(GPIO3_HI, direct_buf, 6, timeout);
// Process all devices' direct sensors
```

This is faster than individual device reads.

---

## Estimated Implementation Time

| Task | Time Estimate |
|------|--------------|
| Update config.h | 2 minutes |
| Fix GPIO configuration in bq_comm.cpp | 5 minutes |
| Update function signature in bq_comm.cpp and .h | 5 minutes |
| Rewrite bqGetAllTemperatures() | 1-2 hours |
| Initial testing and debugging | 1 hour |
| Temperature calibration verification | 30 minutes |
| **Total** | **3-4 hours** |

This assumes familiarity with C++ and the BQ79616 datasheet.

---

## Success Criteria

Your implementation is successful when:

- [✓] Code compiles with no errors or warnings
- [✓] GPIO_CONF3 reads back as 0x29
- [✓] GPIO_CONF4 reads back as 0x2D
- [✓] GPIO6/7/8 show correct bit patterns for each channel
- [✓] All 19 sensors return valid temperature values (not 0 or 255)
- [✓] Temperatures are within ±2°C of known references
- [✓] Readings are stable (don't jump between scans)
- [✓] No communication errors or timeouts
- [✓] Total scan time is under 100ms for both devices

---

## Additional Resources

**BQ79616 Datasheet Sections to Review:**
- Section 7.3: GPIO Configuration
- Section 7.5: Temperature Measurement
- Section 8.6: GPIO_CONF Registers
- Section 8.8: GPIO Measurement Registers

**Key Datasheet Values:**
- GPIO ADC resolution: 16-bit
- GPIO LSB value: 152.59 µV
- GPIO input range: 0-5V
- TSREF output: 5.0V ±2%

**MUX IC Datasheets:**
Check your specific MUX IC datasheet for:
- Switching time (tON, tOFF)
- Propagation delay
- Settling time
- Input voltage range
- On-resistance

---

## Support and Troubleshooting

If you encounter issues:

1. **Re-read this guide** - Most problems are covered in Common Mistakes section
2. **Check hardware connections** - Verify MUX wiring with multimeter
3. **Enable debug logging** - Use BMS_DEBUG_PRINTF throughout
4. **Test incrementally** - Don't implement everything at once
5. **Verify with scope** - Visual confirmation of signals is invaluable

**Good luck with the implementation!** The logic is straightforward but requires careful attention to detail with loops, indexing, and timing.
