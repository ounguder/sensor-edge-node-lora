# Gravimetric Moisture Regression for Blade-Type Analog Sensors

## Objective
Convert measured sensor voltage (`SM_V`) into approximate gravimetric moisture content (`%`) using a linear model derived from your calibration data.

## Calibration Data

### 1) Air/Water Immersion Check (as provided)
This dataset is useful for sanity checks and sensor behavior trend, but the regression below is built from the gravimetric soil-mixture dataset.

| Sensor | Air (V) | Depth 20 (V) | Depth 50 (V) | Depth 70/75 (V) | Depth 100 (V, recommended) |
|---|---:|---:|---:|---:|---:|
| MS-1  | 2.93 | 2.73 | 1.45 | 1.16 | 0.49-0.51 |
| MS-2  | 2.94 | 2.78 | 1.47 | 1.15 | 0.47 |
| MS-7  | 2.93 | 2.76 | 1.46 | 1.17 | 0.49 |
| MS-4  | 2.97 | 2.80 | 1.46 | 1.20 | 0.49 |
| MS-5  | 2.97 | 2.80 | 1.51 | 1.16 | 0.48 |
| MS-6  | 2.88 | 2.72 | 1.48 | 1.16 | 0.45 |
| MS-10 | 2.93 | 2.77 | 1.49 | 1.19 | 0.48 |

Note: You mentioned probe markings in mm and depth entries in cm in text; values above are recorded exactly from your message.

### 2) Gravimetric Soil-Mixture Dataset (used for regression)
Moisture classes: `0%`, `25%`, `50%`, `75%`, `100%`.

| Sensor | 0% (V) | 25% (V) | 50% (V) | 75% (V) | 100% (V) |
|---|---:|---:|---:|---:|---:|
| MS-1  | 2.884 | 1.567 | 1.388 | 1.050 | 0.563 |
| MS-2  | 2.857 | 1.623 | 1.357 | 1.053 | 0.572 |
| MS-7  | 2.849 | 1.573 | 1.364 | 1.131 | 0.587 |
| MS-4  | 2.898 | 1.507 | 1.375 | 1.161 | 0.570 |
| MS-5  | 2.876 | 1.502 | 1.390 | 1.276 | 0.577 |
| MS-6  | 2.900 | 1.491 | 1.325 | 1.153 | 0.579 |
| MS-10 | 2.835 | 1.447 | 1.396 | 1.132 | 0.583 |
| MS-9  | 2.850 | 1.480 | 1.394 | 1.115 | 0.571 |
| MS-8  | 2.844 | 1.535 | 1.377 | 1.213 | 0.569 |
| MS-3  | 2.821 | 1.500 | 1.286 | 1.152 | 0.590 |

Total points used: `10 sensors x 5 moisture points = 50 samples`.

## Regression Approach
- Model form: linear ordinary least squares (OLS)
- Target: `GMC%` (gravimetric moisture content percent)
- Input: sensor output voltage `V`
- Fitted model: `GMC% = a + b * V`
- Pooled fit across all sensors (single deployable equation)

## Output Equation

### Primary model (recommended)
`GMC% = 114.608 - 43.252 * V`

Where:
- `V` is the calibrated sensor voltage (`SM_V`) in volts
- `GMC%` should be clamped to `[0, 100]`

### Inverse form (for reference)
`V = 2.48366 - 0.019798 * GMC%`

## Fit Quality (pooled model)
- `R² = 0.8563`
- `RMSE = 13.40 %`
- `MAE = 11.73 %`

Interpretation: this linear model is appropriate for approximate field estimation, but not laboratory-grade precision.

## Firmware Implementation (C++)

```cpp
// Convert moisture sensor voltage (V) to approximate gravimetric moisture content (%).
static float gravimetric_moisture_from_voltage(float sm_v) {
  float gmc_pct = 114.608f - 43.252f * sm_v;
  if (gmc_pct < 0.0f) gmc_pct = 0.0f;
  if (gmc_pct > 100.0f) gmc_pct = 100.0f;
  return gmc_pct;
}
```

## Suggested Integration Point
- Use `SM_V` after your per-node moisture voltage trim.
- Compute `GMC%` from `SM_V` using the function above.
- Publish/transmit the computed value as `SM_PCT` in LoRa payloads.

## Notes
- The regression above intentionally uses the gravimetric soil-mixture data (scientifically relevant target).
- If you want higher accuracy, next step is a per-sensor calibration model or nonlinear model (piecewise linear / polynomial) after collecting repeated samples across temperature and soil type.
