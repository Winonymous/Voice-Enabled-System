#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
// #include "Test_Audio/Call.h"

#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <pico/stdio_usb.h>
#include "Microphone/machine_i2s.c"
#include <stdio.h>

// FOr multicore functionalities
#include "pico/multicore.h"
#include "hardware/irq.h"

#define SCK 3
#define WS 4 // needs to be SCK +1
#define SD 6
#define BPS 32 // 24 is not valid in this implementation, but INMP441 outputs 24 bits samples
#define RATE 16000

#define SAMPLE_BUFFER_SIZE 16000

// variables
float features[SAMPLE_BUFFER_SIZE];

const uint LED_PIN = 25;

ei_impulse_result_t result = {nullptr}; 

int inference();

void core1_sio_irq() {
  // Just record the latest entry
  while (multicore_fifo_rvalid()){
    int core1_rx_val = multicore_fifo_pop_blocking();
    printf("%d \n" ,core1_rx_val);
    
    int output = inference();
    if(output == 1000){
      printf("Error \n");
    } else if (output == 0){
      printf("Call \n");
    } else if (output == 1){
      printf("Doctor \n");
    } else if (output == 2){
      printf("Nothing \n");
    } else {
      printf("Output: %d", output);
    }
  }

  multicore_fifo_clear_irq();
}

void core1_entry() {
  multicore_fifo_clear_irq();
  irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_sio_irq);

  irq_set_enabled(SIO_IRQ_PROC1, true);

    
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  while (1)
    tight_loop_contents();
}

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr)
{
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

int main()
{
  stdio_usb_init();

  stdio_init_all();
  printf("Hello, multicore_fifo_irqs!\n");

  // We MUST start the other core before we enabled FIFO interrupts.
  // This is because the launch uses the FIFO's, enabling interrupts before
  // they are used for the launch will result in unexpected behaviour.
  multicore_launch_core1(core1_entry);

  machine_i2s_obj_t* i2s0 = machine_i2s_make_new(0, SCK, WS, SD, RX, BPS, MONO, /*ringbuf_len*/SIZEOF_DMA_BUFFER_IN_BYTES, RATE);
	int32_t buffer[I2S_RX_FRAME_SIZE_IN_BYTES /4];

  while (1){
    // Send something back to the other core
    for(int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
			machine_i2s_stream_read(i2s0, (void*)&buffer[0], I2S_RX_FRAME_SIZE_IN_BYTES);
			features[i] = buffer[0]/2000; // right channel is empty, play using $ cat /dev/ttyACM0 | xxd -r -p | aplay -r16000 -c1 -fS32_BE
		}

    multicore_fifo_push_blocking(123);

    // Wait for a bit for things to happen
    sleep_ms(10);
  }
}

int inference(){
  ei_printf("Edge Impulse standalone inferencing (Raspberry Pi Pico)\n");

  if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
  {
    ei_printf("The size of your 'features' array is not correct. Expected %d items, but had %u\n",
              EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
    return 1000;
  }

  // blink LED
  gpio_put(LED_PIN, !gpio_get(LED_PIN));

  // the features are stored into flash, and we don't want to load everything into RAM
  signal_t features_signal;
  features_signal.total_length = sizeof(features) / sizeof(features[0]);
  features_signal.get_data = &raw_feature_get_data;

  // invoke the impulse
  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);

  ei_printf("run_classifier returned: %d\n", res);

  if (res != 0)
    return 1000;

  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

  // print the predictions
  ei_printf("[");
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
  {
    ei_printf("%.5f", result.classification[ix].value);
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf(", ");
#else
    if (ix != EI_CLASSIFIER_LABEL_COUNT - 1)
    {
      ei_printf(", ");
    }
#endif
  }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  printf("%.3f", result.anomaly);
#endif
  printf("]\n");

  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
  {
    if(result.classification[ix].value > 0.5){
      return ix;
    }
  }

  return 1000;
}