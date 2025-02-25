/**
 * High-Performance ADC Reader with Oversampling
 * 
 * Reads ADC as fast as possible, averages values, and sends at fixed intervals.
 * This improves noise performance while maintaining low latency.
 */

#define ADC_PIN           26    // GPIO26 corresponds to ADC0
#define SEND_INTERVAL_MS  1     // Send data every 1ms (1000Hz)
#define BAUD_RATE         460800

// Variables for ADC reading and averaging
unsigned long lastSendTime = 0;
uint32_t adcSum = 0;          // Sum of all readings in current interval
uint16_t sampleCount = 0;     // Number of samples taken in current interval
uint16_t avgValue = 0;        // Last calculated average

void setup() {
  Serial.begin(BAUD_RATE);
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  
  // Optional: Set ADC to free-running mode using direct register access
  // This is RP2040 specific and maximizes ADC throughput
  // The standard Arduino analogRead is still very fast though
  
  lastSendTime = millis();
}

void loop() {
  // Take as many ADC readings as possible
  adcSum += analogRead(ADC_PIN);
  sampleCount++;
  
  // Check if it's time to send data
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= SEND_INTERVAL_MS) {
    // Calculate average (avoid division by zero)
    if (sampleCount > 0) {
      avgValue = adcSum / sampleCount;
    }
    
    // Send the averaged value
    Serial.println(avgValue);
    // Reset for next interval
    adcSum = 0;
    sampleCount = 0;
    lastSendTime = currentTime;
  }
  
  // No delay - read as fast as possible
}