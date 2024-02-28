void reportHeap()
{
  Serial.printf("FreeHeap: %d / HeapLowestPoint: %d / LargestBlock: %d / Used PSRAM: %d\r\n",
                ESP.getFreeHeap(), xPortGetMinimumEverFreeHeapSize(),
                heap_caps_get_largest_free_block(MALLOC_CAP_8BIT), ESP.getPsramSize() - ESP.getFreePsram());
}

// Initialize PSRAM
void psramBegin()
{
  if (psramInit() == false)
  {
    Serial.println("PSRAM not available or is not enabled. This example requires PSRAM. Freezing.");
    while (1);
  }
  if (ESP.getPsramSize() == 0)
  {
    Serial.println("PSRAM not available or is not enabled. This example requires PSRAM. Freezing.");
    while (1);
  }

  Serial.printf("PSRAM Size (bytes): %d\r\n", ESP.getPsramSize());

  heap_caps_malloc_extmem_enable(1000); // Use PSRAM for memory requests larger than 1,000 bytes
}
