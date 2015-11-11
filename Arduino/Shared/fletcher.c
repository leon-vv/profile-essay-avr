#include <stdint.h>

// Computes fletcher's checksum of the
// array passed as the first argument and
// stores the checksum in the second argument.
void 
compute_checksum(uint8_t bytes[3], uint8_t sum[2]) {
    uint8_t first_sum = 0;
    uint8_t second_sum = 0;
    
    for(int i = 0; i < 3; i++) {
        first_sum += bytes[i];
        second_sum += first_sum;
    }

    sum[0] = first_sum;
    sum[1] = second_sum;
}
