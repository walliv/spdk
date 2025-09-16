#ifndef BAR_TESTS_H_
#define BAR_TESTS_H_

void test_bar_accesses(volatile void *bar_base, size_t size) {
    size_t i;

    // 8-bit access
    printf("Testing 8-bit access...\n");
    for (i = 0; i < size; ++i) {
        ((volatile uint8_t *)bar_base)[i] = (uint8_t)(i & 0xFF);
    }
    for (i = 0; i < size; ++i) {
        uint8_t val = ((volatile uint8_t *)bar_base)[i];
        if (val != (uint8_t)(i & 0xFF)) {
            printf("8-bit mismatch at offset %zu: expected 0x%02X, got 0x%02X\n",
                   i, (uint8_t)(i & 0xFF), val);
            return;
        }
    }

    // 16-bit access
    printf("Testing 16-bit access...\n");
    for (i = 0; i + 1 < size; i += 2) {
        ((volatile uint16_t *)bar_base)[i / 2] = (uint16_t)(i & 0xFFFF);
    }
    for (i = 0; i + 1 < size; i += 2) {
        uint16_t val = ((volatile uint16_t *)bar_base)[i / 2];
        if (val != (uint16_t)(i & 0xFFFF)) {
            printf("16-bit mismatch at offset %zu: expected 0x%04X, got 0x%04X\n",
                   i, (uint16_t)(i & 0xFFFF), val);
            return;
        }
    }

    // 32-bit access
    printf("Testing 32-bit access...\n");
    for (i = 0; i + 3 < size; i += 4) {
        ((volatile uint32_t *)bar_base)[i / 4] = (uint32_t)(i ^ 0xA5A5A5A5);
    }
    for (i = 0; i + 3 < size; i += 4) {
        uint32_t val = ((volatile uint32_t *)bar_base)[i / 4];
        if (val != (uint32_t)(i ^ 0xA5A5A5A5)) {
            printf("32-bit mismatch at offset %zu: expected 0x%08X, got 0x%08X\n",
                   i, (uint32_t)(i ^ 0xA5A5A5A5), val);
            return;
        }
    }

    // 64-bit access
    printf("Testing 64-bit access...\n");
    for (i = 0; i + 7 < size; i += 8) {
        ((volatile uint64_t *)bar_base)[i / 8] = (uint64_t)(0xDEADBEEF00000000ULL | (i & 0xFFFFFFFF));
    }
    for (i = 0; i + 7 < size; i += 8) {
        uint64_t val = ((volatile uint64_t *)bar_base)[i / 8];
        uint64_t expected = 0xDEADBEEF00000000ULL | (i & 0xFFFFFFFF);
        if (val != expected) {
            printf("64-bit mismatch at offset %zu: expected 0x%016lX, got 0x%016lX\n",
                   i, expected, val);
            return;
        }
    }

    printf("All access width tests passed successfully.\n");
}

void test_bar_space(uint8_t *bar, size_t size)
{
    size_t i;
    uint8_t pattern[] = {0xAA, 0x55, 0xFF, 0x00};

    // Pattern test
    for (long unsigned p = 0; p < sizeof(pattern); ++p) {
        for (i = 0; i < size; ++i) {
            bar[i] = pattern[p];
        }
        for (i = 0; i < size; ++i) {
            if (bar[i] != pattern[p]) {
                printf("Pattern mismatch at offset %zu: expected 0x%02X, got 0x%02X\n",
                       i, pattern[p], bar[i]);
                return;
            }
        }
    }

    // Walking bit test
    for (uint8_t bit = 1; bit != 0; bit <<= 1) {
        for (i = 0; i < size; ++i) {
            bar[i] = bit;
        }
        for (i = 0; i < size; ++i) {
            if (bar[i] != bit) {
                printf("Walking bit mismatch at offset %zu: expected 0x%02X, got 0x%02X\n",
                       i, bit, bar[i]);
                return;
            }
        }
    }

    // Random test
    srand(42); // Fixed seed for reproducibility
    uint8_t *backup = malloc(size);
    for (i = 0; i < size; ++i) {
        backup[i] = rand() & 0xFF;
        bar[i] = backup[i];
    }
    for (i = 0; i < size; ++i) {
        if (bar[i] != backup[i]) {
            printf("Random mismatch at offset %zu: expected 0x%02X, got 0x%02X\n",
                   i, backup[i], bar[i]);
            free(backup);
            return;
        }
    }
    free(backup);

    printf("BAR test passed successfully.\n");
}


#endif // BAR_TESTS_H_
