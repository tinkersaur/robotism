// TMavc1
// Read left and right long distance ir sensors
// Sharp 6Y2Y)A700

void ReadSensors()
{

  if (adc->startSynchronizedSingleRead(distanceRF, distanceRR)) // reads rght sensors both front and rear
  {
    while (adc->isConverting(ADC_0) || adc->isConverting(ADC_1))
    {
      // wait for conversions to complete
    }
    result = adc->readSynchronizedSingle();
    //    valued1 = result.result_adc0 * 3.3 / adc->getMaxValue(ADC_0);
    //    valued2 = result.result_adc1 * 3.3 / adc->getMaxValue(ADC_1);
    valued1 = map(result.result_adc0, 0, adc->getMaxValue(ADC_0), 0, 5000);
    valued2 = map(result.result_adc1, 0, adc->getMaxValue(ADC_1), 0, 5000);
    rf_Distance = 1.0 / (((valued1 - 1125.0) / 1000.0) / 137.5);
    rr_Distance = 1.0 / (((valued2 - 1125.0) / 1000.0) / 137.5);
    Telemetry.Write[9] = rf_Distance;
    Telemetry.Write[10] = rr_Distance;
    if (IR_DEBUG == 1)
    {
      Serial.print("Pin: ");
      Serial.print(distanceRF);
      Serial.print(", value ADC0: ");
      Serial.println(rf_Distance);
      Serial.print("Pin: ");
      Serial.print(distanceRR);
      Serial.print(", value ADC1: ");
      Serial.println(rr_Distance); 
  } // end right side sensors
  
  if (adc->startSynchronizedSingleRead(distanceLF, distanceLR)) // reads left sensors
  {
    while (adc->isConverting(ADC_0) || adc->isConverting(ADC_1))
    {
      // wait for conversions to complete
    }

    result1 = adc->readSynchronizedSingle();
    valued3 = map(result1.result_adc0, 0, adc->getMaxValue(ADC_0), 0, 5000);
    valued4 = map(result1.result_adc1, 0, adc->getMaxValue(ADC_1), 0, 5000);
    lf_Distance = 1.0 / (((valued3 - 1125.0) / 1000.0) / 137.5);
    lr_Distance = 1.0 / (((valued4 - 1125.0) / 1000.0) / 137.5);
    Telemetry.Write[7] = lf_Distance;
    Telemetry.Write[8] = lr_Distance;
    valued3 = result1.result_adc0 * 3.3 / adc->getMaxValue(ADC_0);
    valued4 = result1.result_adc1 * 3.3 / adc->getMaxValue(ADC_1);
    if (IR_DEBUG == 1)
    {
      Serial.print("Pin: ");
      Serial.print(distanceLF);
      Serial.print(", value ADC0: ");
      Serial.println(valued3);
      Serial.print("Pin: ");
      Serial.print(distanceLR);
      Serial.print(", value ADC1: ");
      Serial.println(valued4);
    }
  }


  /* fail_flag contains all possible errors,
      They are defined in  ADC_Module.h as

      ADC_ERROR_OTHER
      ADC_ERROR_CALIB
      ADC_ERROR_WRONG_PIN
      ADC_ERROR_ANALOG_READ
      ADC_ERROR_COMPARISON
      ADC_ERROR_ANALOG_DIFF_READ
      ADC_ERROR_CONT
      ADC_ERROR_CONT_DIFF
      ADC_ERROR_WRONG_ADC
      ADC_ERROR_SYNCH

      You can compare the value of the flag with those masks to know what's the error.
  */

  if (adc->adc0->fail_flag) {
    Serial.print("ADC0 error flags: 0x");
    Serial.println(adc->adc0->fail_flag, HEX);
    if (adc->adc0->fail_flag == ADC_ERROR_COMPARISON) {
      adc->adc0->fail_flag &= ~ADC_ERROR_COMPARISON; // clear that error
      Serial.println("Comparison error in ADC0");
    }
  }
#if ADC_NUM_ADCS>1
  if (adc->adc1->fail_flag) {
    Serial.print("ADC1 error flags: 0x");
    Serial.println(adc->adc1->fail_flag, HEX);
    if (adc->adc1->fail_flag == ADC_ERROR_COMPARISON) {
      adc->adc1->fail_flag &= ~ADC_ERROR_COMPARISON; // clear that error
      Serial.println("Comparison error in ADC1");
    }
  }
#endif
}
}

// If you enable interrupts make sure to call readSingle() to clear the interrupt.
void adc0_isr() {
  adc->adc0->readSingle();
}




