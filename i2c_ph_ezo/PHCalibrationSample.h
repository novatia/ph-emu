/**
	 *
	 * @author BSIT, Andrea Novati - andrea.novati@n-3.it
	 * @date August 21, 2024
	 * @company N3 S.r.l. - Via Varese, 2 - Saronno (21047) VA
	 * @version 2.0
	 *
	 */
class PHCalibrationSample 
{
  public:
    PHCalibrationSample (float raw, float value ) : rawValue(raw),Value(value){ };
    float rawValue = 0.0f;  // Example raw sensor value at point 
    float Value = 0.0f; // Corresponding calibrated value at point 
};
